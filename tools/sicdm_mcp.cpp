// sicdm-mcp: headless Model Context Protocol server over stdio.
//
// AI agents (Claude Code, Claude Desktop, ...) launch this process and talk
// JSON-RPC over stdin/stdout, one message per line. It edits a project file
// without a window; to drive the live app instead, start SI CDM with
// --mcp-port and use the HTTP transport.
//
//   sicdm-mcp [project.sicdm]

#include "core/CrashReport.hpp"
#include "core/Document.hpp"
#include "core/Mcp.hpp"

#include <cstdio>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    sicdm::installCrashHandler(SICDM_VERSION);
    sicdm::Document doc;
    if (argc > 1) {
        const std::string arg = argv[1];
        if (arg == "--help" || arg == "-h") {
            std::fprintf(stderr, "Usage: %s [project.sicdm]\nMCP server over stdio (JSON-RPC, newline-delimited).\n", argv[0]);
            return 0;
        }
        const sicdm::CommandOutput o = sicdm::executeCommand(doc, "OPEN \"" + arg + "\"", nullptr);
        for (const std::string& l : o.lines) std::fprintf(stderr, "[sicdm-mcp] %s\n", l.c_str());
    }

    sicdm::McpServer server(doc, nullptr);
    std::ios::sync_with_stdio(false);
    std::string line;
    // stdout carries protocol messages only; logs go to stderr.
    while (std::getline(std::cin, line)) {
        if (line.empty() || line == "\r") continue;
        const std::string reply = server.handle(line);
        if (!reply.empty()) {
            std::cout << reply << '\n';
            std::cout.flush();
        }
    }
    return 0;
}
