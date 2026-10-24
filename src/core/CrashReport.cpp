// Crash reports: fault handlers that write a report with the call stack, and the folder
// they go to. The handlers write with the operating system's own calls into buffers set
// up beforehand, so that they work in a process whose heap may be broken.

#include "core/CrashReport.hpp"

#include "core/Recovery.hpp"

#include <algorithm>
#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <exception>
#include <filesystem>
#include <system_error>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <fcntl.h>
#include <unistd.h>
#if defined(__GLIBC__) || defined(__APPLE__)
#include <execinfo.h>
#define SICDM_HAVE_BACKTRACE 1
#endif
#endif

namespace sicdm {

namespace fs = std::filesystem;

std::string crashDirectory() {
    if (const char* d = std::getenv("SICDM_CRASH_DIR"); d && *d) return d;
    const fs::path rec = recoveryDirectory();
#if defined(_WIN32) || defined(__APPLE__)
    return (rec.parent_path() / "Crashes").string();
#else
    return (rec.parent_path() / "crashes").string();
#endif
}

namespace {

// Everything the handler needs, ready before any fault.
char g_dir[1024] = {};
char g_version[64] = {};
char g_context[512] = {};
char g_what[512] = {}; // an uncaught exception's message
std::atomic<int> g_inside{0};
bool g_installed = false;

void copyInto(char* dst, std::size_t cap, const std::string& s) {
    const std::size_t n = std::min(cap - 1, s.size());
    std::memcpy(dst, s.data(), n);
    dst[n] = '\0';
}

// Appends text to a fixed buffer (no allocation).
struct Buf {
    char data[16384];
    std::size_t n = 0;
    void put(const char* s) {
        while (*s && n + 1 < sizeof data) data[n++] = *s++;
    }
    void num(unsigned long long v, int base = 10) {
        char t[32];
        int k = 0;
        do {
            const int d = static_cast<int>(v % static_cast<unsigned>(base));
            t[k++] = static_cast<char>(d < 10 ? '0' + d : 'a' + d - 10);
            v /= static_cast<unsigned>(base);
        } while (v && k < 31);
        if (base == 16) put("0x");
        while (k > 0 && n + 1 < sizeof data) data[n++] = t[--k];
    }
};

const char* signalName(int sig) {
    switch (sig) {
    case SIGSEGV: return "SIGSEGV (bad memory access)";
    case SIGABRT: return "SIGABRT (aborted)";
    case SIGFPE: return "SIGFPE (arithmetic fault)";
    case SIGILL: return "SIGILL (illegal instruction)";
#if !defined(_WIN32)
    case SIGBUS: return "SIGBUS (bus error)";
#endif
    default: return "signal";
    }
}

const char* platformName() {
#if defined(_WIN32)
    return "Windows";
#elif defined(__APPLE__)
    return "macOS";
#else
    return "Linux";
#endif
}

// The report's header: what happened, and where the program was.
void header(Buf& b, const char* what, std::int64_t now) {
    b.put("SI CDM crash report\n");
    b.put("version: "), b.put(g_version), b.put("\n");
    b.put("platform: "), b.put(platformName()), b.put("\n");
    b.put("time: "), b.num(static_cast<unsigned long long>(now)), b.put(" (seconds since 1970, UTC)\n");
    b.put("fault: "), b.put(what), b.put("\n");
    if (g_what[0]) b.put("exception: "), b.put(g_what), b.put("\n");
    if (g_context[0]) b.put("last command: "), b.put(g_context), b.put("\n");
}

// The file: crash-<time>-<pid>.txt in the folder.
void fileName(Buf& p, std::int64_t now, unsigned long pid) {
    p.put(g_dir);
#if defined(_WIN32)
    p.put("\\crash-");
#else
    p.put("/crash-");
#endif
    p.num(static_cast<unsigned long long>(now));
    p.put("-");
    p.num(pid);
    p.put(".txt");
    p.data[p.n] = '\0';
}

#if defined(_WIN32)

void writeReport(const char* what, const void* address, CONTEXT* ctx) {
    (void)ctx;
    if (g_inside.exchange(1)) return;
    const std::int64_t now = static_cast<std::int64_t>(std::time(nullptr));
    static Buf b, p;
    b.n = p.n = 0;
    header(b, what, now);
    if (address) b.put("address: "), b.num(reinterpret_cast<unsigned long long>(address), 16), b.put("\n");
    b.put("\nstack:\n");
    void* frames[62];
    const USHORT count = CaptureStackBackTrace(0, 62, frames, nullptr);
    for (USHORT i = 0; i < count; ++i) {
        HMODULE mod = nullptr;
        char name[MAX_PATH] = "?";
        if (GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, static_cast<LPCSTR>(frames[i]), &mod) &&
            mod)
            GetModuleFileNameA(mod, name, MAX_PATH);
        const char* base = name;
        for (const char* q = name; *q; ++q)
            if (*q == '\\' || *q == '/') base = q + 1;
        b.put("  "), b.num(i), b.put("  "), b.put(base), b.put(" + ");
        b.num(reinterpret_cast<unsigned long long>(frames[i]) - reinterpret_cast<unsigned long long>(mod), 16);
        b.put("\n");
    }
    fileName(p, now, GetCurrentProcessId());
    const HANDLE f = CreateFileA(p.data, GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (f != INVALID_HANDLE_VALUE) {
        DWORD written = 0;
        WriteFile(f, b.data, static_cast<DWORD>(b.n), &written, nullptr);
        CloseHandle(f);
    }
}

const char* exceptionName(DWORD code) {
    switch (code) {
    case 0xE06D7363: // MSVC's C++ exceptions
    case 0x20474343: // GCC's (MinGW): 'GCC' and the kind of unwind
    case 0x21474343:
    case 0x22474343: return "uncaught C++ exception";
    case EXCEPTION_ACCESS_VIOLATION: return "access violation (bad memory access)";
    case EXCEPTION_STACK_OVERFLOW: return "stack overflow";
    case EXCEPTION_INT_DIVIDE_BY_ZERO: return "integer divide by zero";
    case EXCEPTION_ILLEGAL_INSTRUCTION: return "illegal instruction";
    case EXCEPTION_ARRAY_BOUNDS_EXCEEDED: return "array bounds exceeded";
    case EXCEPTION_IN_PAGE_ERROR: return "in-page error";
    default: return "unhandled exception";
    }
}

LONG WINAPI onException(EXCEPTION_POINTERS* e) {
    writeReport(exceptionName(e->ExceptionRecord->ExceptionCode), e->ExceptionRecord->ExceptionAddress, e->ContextRecord);
    return EXCEPTION_CONTINUE_SEARCH;
}

void onSignal(int sig) {
    writeReport(signalName(sig), nullptr, nullptr);
    std::signal(sig, SIG_DFL);
    std::raise(sig);
}

#else

void onSignal(int sig) {
    if (!g_inside.exchange(1)) {
        const std::int64_t now = static_cast<std::int64_t>(std::time(nullptr));
        static Buf b, p;
        header(b, signalName(sig), now);
        b.put("\nstack:\n");
        fileName(p, now, static_cast<unsigned long>(getpid()));
        const int fd = open(p.data, O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (fd >= 0) {
            ssize_t w = write(fd, b.data, b.n);
            (void)w;
#if defined(SICDM_HAVE_BACKTRACE)
            void* frames[64];
            const int count = backtrace(frames, 64);
            backtrace_symbols_fd(frames, count, fd);
#endif
            close(fd);
        }
    }
    // The default action now (the handler was installed to reset itself).
    std::signal(sig, SIG_DFL);
    raise(sig);
}

#endif

void onTerminate() {
    // An exception nobody caught: keep its message for the report, then abort.
    if (std::exception_ptr e = std::current_exception()) {
        try {
            std::rethrow_exception(e);
        } catch (const std::exception& x) {
            copyInto(g_what, sizeof g_what, x.what());
        } catch (...) {
            copyInto(g_what, sizeof g_what, "(not a std::exception)");
        }
    } else copyInto(g_what, sizeof g_what, "std::terminate called");
    std::abort();
}

} // namespace

void installCrashHandler(const std::string& version, const std::string& dir) {
    copyInto(g_version, sizeof g_version, version);
    const std::string d = dir.empty() ? crashDirectory() : dir;
    std::error_code ec;
    fs::create_directories(d, ec);
    copyInto(g_dir, sizeof g_dir, d);
    if (g_installed) return;
    g_installed = true;
    std::set_terminate(onTerminate);
#if defined(_WIN32)
    SetUnhandledExceptionFilter(onException);
    for (int sig : {SIGSEGV, SIGABRT, SIGFPE, SIGILL}) std::signal(sig, onSignal);
#else
    struct sigaction sa;
    std::memset(&sa, 0, sizeof sa);
    sa.sa_handler = onSignal;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = static_cast<int>(SA_RESETHAND);
    for (int sig : {SIGSEGV, SIGABRT, SIGFPE, SIGILL, SIGBUS}) sigaction(sig, &sa, nullptr);
#endif
}

void setCrashContext(const std::string& what) { copyInto(g_context, sizeof g_context, what); }

void crashOnUncaught(const char* what) {
    copyInto(g_what, sizeof g_what, what ? what : "(unknown exception)");
    std::abort();
}

std::vector<CrashReportFile> crashReports(const std::string& dir) {
    std::vector<CrashReportFile> out;
    std::error_code ec;
    for (const auto& e : fs::directory_iterator(dir, ec)) {
        const std::string name = e.path().filename().string();
        if (name.rfind("crash-", 0) != 0 || e.path().extension() != ".txt") continue;
        CrashReportFile f;
        f.path = e.path().string();
        f.seen = name.find(".seen.") != std::string::npos;
        f.time = std::atoll(name.c_str() + 6);
        out.push_back(f);
    }
    std::sort(out.begin(), out.end(), [](const CrashReportFile& a, const CrashReportFile& b) { return a.time != b.time ? a.time > b.time : a.path > b.path; });
    return out;
}

void acknowledgeCrashReports(const std::string& dir) {
    std::error_code ec;
    std::vector<CrashReportFile> all = crashReports(dir);
    for (CrashReportFile& f : all) {
        if (f.seen) continue;
        fs::path p = f.path;
        const fs::path to = p.parent_path() / (p.stem().string() + ".seen.txt");
        fs::rename(p, to, ec);
    }
    all = crashReports(dir);
    for (std::size_t i = 20; i < all.size(); ++i) fs::remove(all[i].path, ec);
}

void clearCrashReports(const std::string& dir) {
    std::error_code ec;
    for (const CrashReportFile& f : crashReports(dir)) fs::remove(f.path, ec);
}

} // namespace sicdm
