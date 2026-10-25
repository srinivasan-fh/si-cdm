// Plugins: manifests, the programs' processes, and the JSON-RPC conversation with them.

#include "core/Plugin.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Json.hpp"
#include "core/Mcp.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstring>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
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
#include <csignal>
#include <fcntl.h>
#include <poll.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace sicdm {

namespace fs = std::filesystem;

namespace {

std::string env(const char* k) {
    const char* v = std::getenv(k);
    return v ? v : "";
}

std::string upper(std::string s) {
    for (char& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

std::vector<std::string> strings(const Json& a) {
    std::vector<std::string> out;
    for (const Json& v : a.items())
        if (v.isString()) out.push_back(v.asString());
    return out;
}

} // namespace

std::string pluginsFolder() {
#if defined(_WIN32)
    if (const std::string d = env("APPDATA"); !d.empty()) return (fs::path(d) / "SI-CDM" / "Plugins").string();
#elif defined(__APPLE__)
    if (const std::string h = env("HOME"); !h.empty()) return (fs::path(h) / "Library" / "Application Support" / "SI-CDM" / "Plugins").string();
#else
    if (const std::string d = env("XDG_CONFIG_HOME"); !d.empty()) return (fs::path(d) / "sicdm" / "plugins").string();
    if (const std::string h = env("HOME"); !h.empty()) return (fs::path(h) / ".config" / "sicdm" / "plugins").string();
#endif
    return {};
}

bool loadPlugin(const std::string& dir, Plugin& out) {
    out = Plugin{};
    out.dir = dir;
    out.name = fs::path(dir).filename().string();
    std::ifstream f(fs::path(dir) / "plugin.json", std::ios::binary);
    if (!f) {
        out.error = "no plugin.json";
        return false;
    }
    std::ostringstream s;
    s << f.rdbuf();
    std::string err;
    const auto j = Json::parse(s.str(), &err);
    if (!j || !j->isObject()) {
        out.error = "plugin.json isn't a JSON object" + (err.empty() ? std::string() : ": " + err);
        return false;
    }
    const Json& m = *j;
    if (m["name"].isString()) out.name = m["name"].asString();
    out.version = m["version"].asString(std::string());
    out.description = m["description"].asString(std::string());
    out.author = m["author"].asString(std::string());
    out.timeout = std::clamp(m["timeout"].asNumber(120.0), 0.1, 24.0 * 3600.0);
    for (const Json& c : m["commands"].items()) {
        PluginCommand pc;
        pc.name = upper(c["name"].asString(std::string()));
        pc.usage = c["usage"].asString(pc.name);
        pc.help = c["help"].asString(std::string());
#if defined(_WIN32)
        pc.run = strings(c["run_windows"].isArray() ? c["run_windows"] : c["run"]);
#elif defined(__APPLE__)
        pc.run = strings(c["run_macos"].isArray() ? c["run_macos"] : c["run"]);
#else
        pc.run = strings(c["run_linux"].isArray() ? c["run_linux"] : c["run"]);
#endif
        if (pc.name.empty() || pc.run.empty()) {
            out.error = "a command without a name or a \"run\"";
            continue;
        }
        bool word = true;
        for (char ch : pc.name) word = word && (std::isalnum(static_cast<unsigned char>(ch)) || ch == '_');
        if (!word) {
            out.error = "the command " + pc.name + " isn't one word of letters, digits and _";
            continue;
        }
        if (findCommand(pc.name)) {
            out.error = "the command " + pc.name + " is built in";
            continue;
        }
        out.commands.push_back(pc);
    }
    for (const Json& mi : m["menu"].items())
        if (mi["label"].isString() && mi["command"].isString()) out.menu.push_back({mi["label"].asString(), mi["command"].asString()});
    if (out.commands.empty() && out.error.empty()) out.error = "no commands";
    return !out.commands.empty();
}

namespace {

std::vector<Plugin>& cache() {
    static std::vector<Plugin> c;
    return c;
}
bool& loaded() {
    static bool l = false;
    return l;
}

} // namespace

void reloadPlugins() {
    std::vector<Plugin>& all = cache();
    all.clear();
    std::vector<std::string> roots;
    std::string path = env("SICDM_PLUGIN_PATH");
#if defined(_WIN32)
    const char sep = ';';
#else
    const char sep = ':';
#endif
    std::size_t at = 0;
    while (at <= path.size() && !path.empty()) {
        const std::size_t next = path.find(sep, at);
        const std::string part = path.substr(at, next == std::string::npos ? std::string::npos : next - at);
        if (!part.empty()) roots.push_back(part);
        if (next == std::string::npos) break;
        at = next + 1;
    }
    if (const std::string u = pluginsFolder(); !u.empty()) roots.push_back(u);
    for (const std::string& root : roots) {
        std::error_code ec;
        std::vector<fs::path> dirs;
        if (fs::exists(fs::path(root) / "plugin.json", ec)) dirs.push_back(root); // the folder is a plugin itself
        else
            for (const auto& e : fs::directory_iterator(root, ec))
                if (e.is_directory(ec) && fs::exists(e.path() / "plugin.json", ec)) dirs.push_back(e.path());
        std::sort(dirs.begin(), dirs.end());
        for (const fs::path& d : dirs) {
            Plugin p;
            loadPlugin(d.string(), p);
            // A command another plugin already has: the first one found keeps it.
            for (auto it = p.commands.begin(); it != p.commands.end();) {
                const Plugin* other = nullptr;
                bool taken = false;
                for (const Plugin& q : all)
                    for (const PluginCommand& c : q.commands)
                        if (c.name == it->name) taken = true, other = &q;
                if (taken) {
                    p.error = "the command " + it->name + " is " + other->name + "'s";
                    it = p.commands.erase(it);
                } else ++it;
            }
            all.push_back(std::move(p));
        }
    }
    loaded() = true;
}

const std::vector<Plugin>& plugins() {
    if (!loaded()) reloadPlugins();
    return cache();
}

const PluginCommand* findPluginCommand(const std::string& name, const Plugin** owner) {
    const std::string n = upper(name);
    for (const Plugin& p : plugins())
        for (const PluginCommand& c : p.commands)
            if (c.name == n) {
                if (owner) *owner = &p;
                return &c;
            }
    return nullptr;
}

// --- the program's process --------------------------------------------------------------------

namespace {

// A child process with pipes to its standard input and output; its standard error goes to a file.
class Child {
public:
    ~Child() { stop(); }

    bool start(const std::vector<std::string>& argv, const std::string& dir, const std::string& errFile, std::string* error) {
#if defined(_WIN32)
        auto quoteArg = [](const std::string& a) {
            if (!a.empty() && a.find_first_of(" \t\"") == std::string::npos) return a;
            std::string q = "\"";
            std::size_t backslashes = 0;
            for (char c : a) {
                if (c == '\\') ++backslashes;
                else if (c == '"') q.append(backslashes + 1, '\\'), backslashes = 0; // the n already copied, doubled, and one for the quote
                else backslashes = 0;
                if (c != '"') q += c;
                else q += '"';
            }
            q.append(backslashes, '\\');
            return q + "\"";
        };
        std::string cmd;
        for (const std::string& a : argv) cmd += (cmd.empty() ? "" : " ") + quoteArg(a);
        SECURITY_ATTRIBUTES sa{sizeof(SECURITY_ATTRIBUTES), nullptr, TRUE};
        HANDLE inRead = nullptr, inWrite = nullptr, outRead = nullptr, outWrite = nullptr;
        if (!CreatePipe(&inRead, &inWrite, &sa, 0) || !CreatePipe(&outRead, &outWrite, &sa, 0)) {
            if (error) *error = "Can't make pipes for the plugin";
            return false;
        }
        SetHandleInformation(inWrite, HANDLE_FLAG_INHERIT, 0);
        SetHandleInformation(outRead, HANDLE_FLAG_INHERIT, 0);
        HANDLE err = CreateFileA(errFile.c_str(), GENERIC_WRITE, FILE_SHARE_READ, &sa, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
        STARTUPINFOA si{};
        si.cb = sizeof si;
        si.dwFlags = STARTF_USESTDHANDLES;
        si.hStdInput = inRead;
        si.hStdOutput = outWrite;
        si.hStdError = err != INVALID_HANDLE_VALUE ? err : outWrite;
        PROCESS_INFORMATION pi{};
        SetEnvironmentVariableA("SICDM_PLUGIN_DIR", dir.c_str());
        SetEnvironmentVariableA("SICDM_VERSION", SICDM_VERSION);
        std::vector<char> line(cmd.begin(), cmd.end());
        line.push_back('\0');
        const BOOL ok = CreateProcessA(nullptr, line.data(), nullptr, nullptr, TRUE, CREATE_NO_WINDOW, nullptr, nullptr, &si, &pi);
        CloseHandle(inRead);
        CloseHandle(outWrite);
        if (err != INVALID_HANDLE_VALUE) CloseHandle(err);
        if (!ok) {
            CloseHandle(inWrite);
            CloseHandle(outRead);
            if (error) *error = "Can't start " + argv[0] + " (error " + std::to_string(GetLastError()) + ")";
            return false;
        }
        CloseHandle(pi.hThread);
        m_process = pi.hProcess, m_in = inWrite, m_out = outRead;
        return true;
#else
        int in[2], out[2];
        if (pipe(in) != 0 || pipe(out) != 0) {
            if (error) *error = "Can't make pipes for the plugin";
            return false;
        }
        // An error from exec reaches the parent through a pipe closed on a successful exec.
        int report[2];
        if (pipe(report) != 0) {
            if (error) *error = "Can't make pipes for the plugin";
            return false;
        }
        fcntl(report[1], F_SETFD, FD_CLOEXEC);
        std::vector<char*> args;
        for (const std::string& a : argv) args.push_back(const_cast<char*>(a.c_str()));
        args.push_back(nullptr);
        const pid_t pid = fork();
        if (pid < 0) {
            if (error) *error = "Can't start the plugin";
            return false;
        }
        if (pid == 0) {
            dup2(in[0], 0);
            dup2(out[1], 1);
            const int err = open(errFile.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0600);
            if (err >= 0) dup2(err, 2);
            close(in[0]), close(in[1]), close(out[0]), close(out[1]), close(report[0]);
            setenv("SICDM_PLUGIN_DIR", dir.c_str(), 1);
            setenv("SICDM_VERSION", SICDM_VERSION, 1);
            execvp(args[0], args.data());
            const int e = errno;
            ssize_t w = write(report[1], &e, sizeof e);
            (void)w;
            _exit(127);
        }
        close(in[0]), close(out[1]), close(report[1]);
        int e = 0;
        const ssize_t got = read(report[0], &e, sizeof e);
        close(report[0]);
        m_pid = pid, m_in = in[1], m_out = out[0];
        if (got == static_cast<ssize_t>(sizeof e)) {
            if (error) *error = "Can't start " + argv[0] + ": " + std::strerror(e);
            return false;
        }
        signal(SIGPIPE, SIG_IGN); // a plugin that stops reading mustn't kill SI CDM
        return true;
#endif
    }

    // A line from the program, waiting until `deadline`. False at its end, or on time out (then timedOut).
    bool readLine(std::string& line, std::chrono::steady_clock::time_point deadline, bool& timedOut) {
        timedOut = false;
        for (;;) {
            const std::size_t nl = m_buf.find('\n');
            if (nl != std::string::npos) {
                line = m_buf.substr(0, nl);
                if (!line.empty() && line.back() == '\r') line.pop_back();
                m_buf.erase(0, nl + 1);
                return true;
            }
            if (m_eof) {
                if (m_buf.empty()) return false;
                line.swap(m_buf);
                m_buf.clear();
                return true;
            }
            const auto now = std::chrono::steady_clock::now();
            if (now >= deadline) {
                timedOut = true;
                return false;
            }
            char chunk[4096];
#if defined(_WIN32)
            DWORD avail = 0;
            if (!PeekNamedPipe(m_out, nullptr, 0, nullptr, &avail, nullptr)) {
                m_eof = true;
                continue;
            }
            if (avail == 0) {
                Sleep(2);
                continue;
            }
            DWORD n = 0;
            if (!ReadFile(m_out, chunk, std::min<DWORD>(avail, sizeof chunk), &n, nullptr) || n == 0) m_eof = true;
            else m_buf.append(chunk, n);
#else
            pollfd p{m_out, POLLIN, 0};
            const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();
            const int r = poll(&p, 1, static_cast<int>(std::min<long long>(ms, 1000)));
            if (r <= 0) continue;
            const ssize_t n = read(m_out, chunk, sizeof chunk);
            if (n <= 0) m_eof = true;
            else m_buf.append(chunk, static_cast<std::size_t>(n));
#endif
        }
    }

    bool writeLine(const std::string& s) {
        const std::string data = s + "\n";
#if defined(_WIN32)
        DWORD n = 0;
        return WriteFile(m_in, data.data(), static_cast<DWORD>(data.size()), &n, nullptr) && n == data.size();
#else
        std::size_t done = 0;
        while (done < data.size()) {
            const ssize_t n = write(m_in, data.data() + done, data.size() - done);
            if (n <= 0) return false;
            done += static_cast<std::size_t>(n);
        }
        return true;
#endif
    }

    // Closes its input and waits for it (at most until `deadline`, then stops it). The exit code.
    int finish(std::chrono::steady_clock::time_point deadline, bool& timedOut) {
        timedOut = false;
#if defined(_WIN32)
        if (m_in) CloseHandle(m_in), m_in = nullptr;
        if (!m_process) return -1;
        const auto ms = std::max<long long>(0, std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now()).count());
        if (WaitForSingleObject(m_process, static_cast<DWORD>(ms)) == WAIT_TIMEOUT) {
            timedOut = true;
            stop();
            return -1;
        }
        DWORD code = 0;
        GetExitCodeProcess(m_process, &code);
        CloseHandle(m_process), m_process = nullptr;
        if (m_out) CloseHandle(m_out), m_out = nullptr;
        return static_cast<int>(code);
#else
        if (m_in >= 0) close(m_in), m_in = -1;
        if (m_pid <= 0) return -1;
        for (;;) {
            int status = 0;
            const pid_t r = waitpid(m_pid, &status, WNOHANG);
            if (r == m_pid) {
                m_pid = -1;
                if (m_out >= 0) close(m_out), m_out = -1;
                return WIFEXITED(status) ? WEXITSTATUS(status) : 128 + (WIFSIGNALED(status) ? WTERMSIG(status) : 0);
            }
            if (std::chrono::steady_clock::now() >= deadline) {
                timedOut = true;
                stop();
                return -1;
            }
            usleep(2000);
        }
#endif
    }

    void stop() {
#if defined(_WIN32)
        if (m_process) {
            TerminateProcess(m_process, 1);
            WaitForSingleObject(m_process, 2000);
            CloseHandle(m_process), m_process = nullptr;
        }
        if (m_in) CloseHandle(m_in), m_in = nullptr;
        if (m_out) CloseHandle(m_out), m_out = nullptr;
#else
        if (m_pid > 0) {
            kill(m_pid, SIGKILL);
            int status = 0;
            waitpid(m_pid, &status, 0);
            m_pid = -1;
        }
        if (m_in >= 0) close(m_in), m_in = -1;
        if (m_out >= 0) close(m_out), m_out = -1;
#endif
    }

private:
#if defined(_WIN32)
    HANDLE m_process = nullptr, m_in = nullptr, m_out = nullptr;
#else
    pid_t m_pid = -1;
    int m_in = -1, m_out = -1;
#endif
    std::string m_buf;
    bool m_eof = false;
};

std::string tail(const std::string& path, std::size_t lines) {
    std::ifstream f(path, std::ios::binary);
    std::vector<std::string> all;
    for (std::string l; std::getline(f, l);) {
        if (!l.empty() && l.back() == '\r') l.pop_back();
        all.push_back(l);
    }
    std::string out;
    for (std::size_t i = all.size() > lines ? all.size() - lines : 0; i < all.size(); ++i) out += (out.empty() ? "" : "\n") + all[i];
    return out;
}

} // namespace

int runProgram(const std::vector<std::string>& argv, double timeout, std::string& output, std::string* error) {
    output.clear();
    if (argv.empty()) return -1;
    std::error_code ec;
    const std::string errFile = (fs::temp_directory_path(ec) / ("sicdm-run-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".err")).string();
    Child child;
    std::string err;
    if (!child.start(argv, fs::current_path(ec).string(), errFile, &err)) {
        fs::remove(errFile, ec);
        if (error) *error = err;
        return -1;
    }
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(static_cast<long long>(timeout * 1000.0));
    bool timedOut = false;
    for (std::string line; child.readLine(line, deadline, timedOut);) output += line + "\n";
    int code = -1;
    if (!timedOut) code = child.finish(deadline, timedOut);
    else child.stop();
    if (timedOut) {
        if (error) *error = argv[0] + " took longer than " + std::to_string(static_cast<int>(timeout)) + " s";
        code = -1;
    } else if (code != 0 && error) {
        *error = tail(errFile, 4);
        if (error->empty()) *error = argv[0] + " exited with " + std::to_string(code);
    }
    fs::remove(errFile, ec);
    return code;
}

CommandOutput runPluginCommand(Document& doc, ViewHost* view, const Plugin& plugin, const PluginCommand& command, const std::vector<std::string>& args) {
    CommandOutput out;
    std::vector<std::string> argv = command.run;
    // The program: a file in the plugin's folder, else from the PATH.
    std::error_code ec;
    if (const fs::path local = fs::path(plugin.dir) / argv[0]; fs::exists(local, ec) && !fs::is_directory(local, ec)) argv[0] = local.string();
    for (std::size_t i = 1; i < argv.size(); ++i) // later words naming the plugin's files (a script for an interpreter)
        if (const fs::path local = fs::path(plugin.dir) / argv[i]; argv[i].find("..") == std::string::npos && fs::exists(local, ec) && !fs::is_directory(local, ec))
            argv[i] = local.string();
    argv.insert(argv.end(), args.begin(), args.end());
    const std::string errFile = (fs::temp_directory_path(ec) / ("sicdm-plugin-" + plugin.name + "-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".err")).string();
    Child child;
    std::string err;
    if (!child.start(argv, plugin.dir, errFile, &err)) {
        fs::remove(errFile, ec);
        return out.fail(plugin.name + ": " + err);
    }
    McpServer server(doc, view);
    server.handle(R"({"jsonrpc":"2.0","id":0,"method":"initialize","params":{"protocolVersion":"2025-06-18","capabilities":{},"clientInfo":{"name":"plugin","version":"1"}}})");
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(static_cast<long long>(plugin.timeout * 1000.0));
    std::string line;
    bool timedOut = false;
    std::size_t calls = 0;
    while (child.readLine(line, deadline, timedOut)) {
        if (line.empty()) continue;
        std::string perr;
        const auto msg = line.front() == '{' ? Json::parse(line, &perr) : std::nullopt;
        if (!msg || !msg->isObject() || !(*msg)["method"].isString()) {
            out.say(line); // plain output
            continue;
        }
        const Json& m = *msg;
        const std::string method = m["method"].asString();
        const bool request = m.has("id");
        if (method == "sicdm/say") {
            const std::string text = m["params"]["text"].asString(std::string());
            std::istringstream in(text);
            for (std::string t; std::getline(in, t);) out.say(t);
            if (request) {
                Json r = Json::object();
                r.set("jsonrpc", "2.0").set("id", m["id"]).set("result", Json::object());
                child.writeLine(r.dump());
            }
            continue;
        }
        ++calls;
        const Json reply = server.handleJson(m);
        if (request && !reply.isNull() && !child.writeLine(reply.dump())) break;
    }
    int code = -1;
    if (!timedOut) code = child.finish(std::chrono::steady_clock::now() + std::chrono::seconds(10), timedOut);
    else child.stop();
    const std::string errors = tail(errFile, 12);
    fs::remove(errFile, ec);
    if (server.takeDocumentReplaced()) out.documentReplaced = true;
    if (timedOut) {
        out.fail(plugin.name + ": " + command.name + " took longer than " + std::to_string(static_cast<int>(plugin.timeout + 0.5)) + " s and was stopped");
        return out;
    }
    if (code != 0) {
        out.fail(plugin.name + ": " + command.name + " failed (exit code " + std::to_string(code) + ")");
        if (!errors.empty()) {
            std::istringstream in(errors);
            for (std::string t; std::getline(in, t);) out.say("  " + t);
        }
        return out;
    }
    if (out.lines.empty()) out.say(command.name + " done (" + std::to_string(calls) + " requests)");
    return out;
}

} // namespace sicdm
