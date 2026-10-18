#include "app/McpHttpServer.hpp"

#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstring>

#if defined(_WIN32)
// The HTTP transport is POSIX-only for now; Windows builds report it unavailable.
namespace sicdm {
bool McpHttpServer::start(int, std::function<void()>, std::string* error) {
    if (error) *error = "MCP HTTP server is not available on Windows yet; use sicdm-mcp (stdio)";
    return false;
}
void McpHttpServer::stop() {}
int McpHttpServer::poll(const Handler&) { return 0; }
void McpHttpServer::serve(int) {}
void McpHttpServer::handleConnection(int) {}
std::string McpHttpServer::dispatch(const std::string&) { return {}; }
} // namespace sicdm
#else

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <unistd.h>

namespace sicdm {

namespace {

constexpr std::size_t kMaxBody = 32 * 1024 * 1024;

#if defined(MSG_NOSIGNAL)
constexpr int kSendFlags = MSG_NOSIGNAL;
#else
constexpr int kSendFlags = 0; // macOS: SO_NOSIGPIPE is set on the socket instead
#endif

std::string lower(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

bool sendAll(int fd, const std::string& data) {
    std::size_t off = 0;
    while (off < data.size()) {
        const ssize_t n = ::send(fd, data.data() + off, data.size() - off, kSendFlags);
        if (n <= 0) return false;
        off += static_cast<std::size_t>(n);
    }
    return true;
}

void respond(int fd, int code, const char* status, const std::string& body, const char* type = "application/json",
             const char* extra = "") {
    std::string r = "HTTP/1.1 " + std::to_string(code) + " " + status + "\r\n";
    r += "Content-Type: " + std::string(type) + "\r\n";
    r += "Content-Length: " + std::to_string(body.size()) + "\r\n";
    r += "Cache-Control: no-store\r\nConnection: close\r\n";
    r += extra;
    r += "\r\n";
    r += body;
    sendAll(fd, r);
}

// Only local pages may call us from a browser (DNS-rebinding protection).
bool originAllowed(const std::string& origin) {
    if (origin.empty() || origin == "null") return true;
    const std::string o = lower(origin);
    for (const char* ok : {"http://localhost", "http://127.0.0.1", "https://localhost", "https://127.0.0.1"})
        if (o.rfind(ok, 0) == 0 && (o.size() == std::strlen(ok) || o[std::strlen(ok)] == ':')) return true;
    return false;
}

} // namespace

bool McpHttpServer::start(int port, std::function<void()> wake, std::string* error) {
    if (m_running) return true;
    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        if (error) *error = std::string("socket: ") + std::strerror(errno);
        return false;
    }
    const int yes = 1;
    ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK); // never exposed to the network
    if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0 || ::listen(fd, 16) != 0) {
        if (error) *error = "Cannot listen on 127.0.0.1:" + std::to_string(port) + ": " + std::strerror(errno);
        ::close(fd);
        return false;
    }
    m_listenFd = fd;
    m_port = port;
    m_wake = std::move(wake);
    m_stop = false;
    m_running = true;
    m_thread = std::thread([this, fd] { serve(fd); });
    return true;
}

void McpHttpServer::stop() {
    if (!m_running) return;
    m_stop = true;
    {
        // Release a request still waiting for the main thread, then join.
        std::lock_guard<std::mutex> lock(m_mutex);
        for (Pending* p : m_queue) p->reply.set_value({});
        m_queue.clear();
    }
    if (m_thread.joinable()) m_thread.join();
    ::close(m_listenFd);
    m_listenFd = -1;
    m_running = false;
}

void McpHttpServer::serve(int listenFd) {
    while (!m_stop) {
        pollfd pfd{listenFd, POLLIN, 0};
        if (::poll(&pfd, 1, 200) <= 0) continue;
        const int fd = ::accept(listenFd, nullptr, nullptr);
        if (fd < 0) continue;
        timeval tv{10, 0};
        ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
#if defined(SO_NOSIGPIPE)
        const int one = 1;
        ::setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof(one));
#endif
        handleConnection(fd);
        ::close(fd);
    }
}

std::string McpHttpServer::dispatch(const std::string& body) {
    Pending p;
    p.body = body;
    auto fut = p.reply.get_future();
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_queue.push_back(&p);
    }
    if (m_wake) m_wake();
    // The main thread answers within a frame; give up after 30 s.
    if (fut.wait_for(std::chrono::seconds(30)) != std::future_status::ready) {
        std::lock_guard<std::mutex> lock(m_mutex);
        for (auto it = m_queue.begin(); it != m_queue.end(); ++it)
            if (*it == &p) {
                m_queue.erase(it);
                return R"({"jsonrpc":"2.0","id":null,"error":{"code":-32603,"message":"App did not respond"}})";
            }
        return fut.get(); // answered at the last moment
    }
    return fut.get();
}

int McpHttpServer::poll(const Handler& handler) {
    int n = 0;
    while (true) {
        Pending* p = nullptr;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            if (m_queue.empty()) break;
            p = m_queue.front();
            m_queue.pop_front();
        }
        p->reply.set_value(handler(p->body));
        ++n;
    }
    return n;
}

void McpHttpServer::handleConnection(int fd) {
    // Read headers.
    std::string req;
    char buf[8192];
    std::size_t headerEnd = std::string::npos;
    while (headerEnd == std::string::npos && req.size() < 64 * 1024) {
        const ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
        if (n <= 0) return;
        req.append(buf, static_cast<std::size_t>(n));
        headerEnd = req.find("\r\n\r\n");
    }
    if (headerEnd == std::string::npos) return respond(fd, 431, "Request Header Fields Too Large", "");

    const std::string head = req.substr(0, headerEnd);
    std::string body = req.substr(headerEnd + 4);
    const std::size_t sp1 = head.find(' '), sp2 = head.find(' ', sp1 + 1);
    if (sp1 == std::string::npos || sp2 == std::string::npos) return respond(fd, 400, "Bad Request", "");
    const std::string method = head.substr(0, sp1);
    std::string target = head.substr(sp1 + 1, sp2 - sp1 - 1);
    if (const std::size_t q = target.find('?'); q != std::string::npos) target.resize(q);

    std::size_t contentLength = 0;
    std::string origin;
    for (std::size_t pos = head.find("\r\n"); pos != std::string::npos;) {
        const std::size_t next = head.find("\r\n", pos + 2);
        const std::string line = head.substr(pos + 2, next == std::string::npos ? std::string::npos : next - pos - 2);
        const std::size_t colon = line.find(':');
        if (colon != std::string::npos) {
            const std::string key = lower(line.substr(0, colon));
            std::string value = line.substr(colon + 1);
            while (!value.empty() && value.front() == ' ') value.erase(0, 1);
            if (key == "content-length") contentLength = static_cast<std::size_t>(std::strtoull(value.c_str(), nullptr, 10));
            if (key == "origin") origin = value;
        }
        pos = next;
    }

    if (!originAllowed(origin)) return respond(fd, 403, "Forbidden", R"({"error":"origin not allowed"})");
    if (target != "/mcp" && target != "/") return respond(fd, 404, "Not Found", R"({"error":"use /mcp"})");
    if (method == "GET" || method == "DELETE")
        return respond(fd, 405, "Method Not Allowed", "", "application/json", "Allow: POST\r\n");
    if (method != "POST") return respond(fd, 405, "Method Not Allowed", "", "application/json", "Allow: POST\r\n");
    if (contentLength > kMaxBody) return respond(fd, 413, "Payload Too Large", "");

    while (body.size() < contentLength) {
        const ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
        if (n <= 0) return;
        body.append(buf, static_cast<std::size_t>(n));
    }
    body.resize(contentLength);

    const std::string reply = dispatch(body);
    if (reply.empty()) return respond(fd, 202, "Accepted", ""); // notification
    respond(fd, 200, "OK", reply);
}

} // namespace sicdm

#endif
