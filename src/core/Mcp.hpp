#pragma once

// Model Context Protocol server: lets AI agents read and edit the 2D / 3D
// document. Transport-agnostic: feed it JSON-RPC messages, get replies.
// Transports: stdio (sicdm-mcp, headless) and HTTP (live app, --mcp-port).

#include "core/Document.hpp"
#include "core/Json.hpp"

#include <string>
#include <string_view>

namespace sicdm {

class McpServer {
public:
    // `view` may be null (headless): window / camera tools then report that.
    McpServer(Document& doc, ViewHost* view) : m_doc(doc), m_view(view) {}

    // One JSON-RPC message in, reply out; empty for notifications.
    std::string handle(std::string_view message);
    // Same, on parsed JSON; null for notifications.
    Json handleJson(const Json& message);

    // Called after a tool changed the document (GUI refreshes on this).
    bool takeDocumentChanged() noexcept {
        const bool c = m_changed;
        m_changed = false;
        return c;
    }
    bool takeDocumentReplaced() noexcept {
        const bool r = m_replaced;
        m_replaced = false;
        return r;
    }

    static Json toolDefinitions();
    Json sceneJson() const;

private:
    Json callTool(const std::string& name, const Json& args);
    Json runLine(const std::string& line); // tool result from a command line

    Document& m_doc;
    ViewHost* m_view;
    bool m_initialized = false;
    bool m_changed = false;
    bool m_replaced = false;
};

} // namespace sicdm
