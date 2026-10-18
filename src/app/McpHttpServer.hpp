#pragma once

// MCP "Streamable HTTP" transport for the live app: POST JSON-RPC to
// http://127.0.0.1:<port>/mcp. Bound to localhost only. The socket runs on a
// background thread; requests are executed on the main thread via poll(),
// so the document is never touched concurrently.

#include <condition_variable>
#include <deque>
#include <functional>
#include <future>
#include <mutex>
#include <string>
#include <thread>

namespace sicdm {

class McpHttpServer {
public:
    using Handler = std::function<std::string(const std::string& body)>;

    ~McpHttpServer() { stop(); }

    // `wake` is called from the socket thread when a request is waiting
    // (e.g. glfwPostEmptyEvent). Returns false and sets `error` on failure.
    bool start(int port, std::function<void()> wake, std::string* error);
    void stop();
    bool running() const noexcept { return m_running; }
    int port() const noexcept { return m_port; }

    // Main thread: run queued requests through `handler`. Returns how many ran.
    int poll(const Handler& handler);

private:
    struct Pending {
        std::string body;
        std::promise<std::string> reply;
    };

    void serve(int listenFd);
    void handleConnection(int fd);
    std::string dispatch(const std::string& body); // blocks until the main thread answers

    std::thread m_thread;
    std::mutex m_mutex;
    std::deque<Pending*> m_queue;
    std::function<void()> m_wake;
    int m_listenFd = -1;
    int m_port = 0;
    volatile bool m_running = false;
    volatile bool m_stop = false;
};

} // namespace sicdm
