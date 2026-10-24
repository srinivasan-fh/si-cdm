// A plugin for the plugin tests: speaks JSON-RPC over its standard input and output.
//   sample_plugin box <size>   makes a box through run_command, says so, reads the scene
//   sample_plugin fail         writes to its standard error and exits with 3
//   sample_plugin hang         never answers
//   sample_plugin echo ...     prints its arguments as plain lines

#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

namespace {

std::string ask(int id, const std::string& method, const std::string& params) {
    std::cout << R"({"jsonrpc":"2.0","id":)" << id << R"(,"method":")" << method << R"(","params":)" << params << "}\n" << std::flush;
    std::string reply;
    std::getline(std::cin, reply);
    return reply;
}

} // namespace

int main(int argc, char** argv) {
    const std::string mode = argc > 1 ? argv[1] : "";
    if (mode == "fail") {
        std::fprintf(stderr, "the sample plugin can't do that\n");
        return 3;
    }
    if (mode == "hang") {
        std::this_thread::sleep_for(std::chrono::seconds(30));
        return 0;
    }
    if (mode == "echo") {
        for (int i = 2; i < argc; ++i) std::cout << "arg: " << argv[i] << "\n";
        std::cout << "dir: " << (std::getenv("SICDM_PLUGIN_DIR") ? std::getenv("SICDM_PLUGIN_DIR") : "") << "\n";
        return 0;
    }
    if (mode == "box") {
        const std::string size = argc > 2 ? argv[2] : "10";
        const std::string r = ask(1, "tools/call", R"({"name":"run_command","arguments":{"command":"BOX )" + size + " " + size + " " + size + R"("}})");
        if (r.find("Created") == std::string::npos) {
            std::fprintf(stderr, "no box: %s\n", r.c_str());
            return 4;
        }
        std::cout << R"({"jsonrpc":"2.0","method":"sicdm/say","params":{"text":"made a )" << size << R"( mm box"}})" << "\n" << std::flush;
        const std::string scene = ask(2, "tools/call", R"({"name":"get_scene","arguments":{}})");
        std::cout << "scene reply " << (scene.find("Block") != std::string::npos ? "has the block" : "lacks the block") << "\n";
        const std::string bad = ask(3, "tools/call", R"({"name":"no_such_tool","arguments":{}})");
        std::cout << (bad.find("error") != std::string::npos ? "unknown tools are errors" : "unknown tools pass") << "\n";
        return 0;
    }
    return 2;
}
