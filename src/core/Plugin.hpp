#pragma once

// Plugins: programs that add commands to SI CDM.
//
// A plugin is a folder with a plugin.json manifest and the program it runs. The program can
// be written in anything (Python, JavaScript, C++, a shell script); it runs in its own
// process, so a plugin that fails or crashes can't take SI CDM down with it.
//
// When one of a plugin's commands is run, SI CDM starts the program with the command's
// arguments and talks to it over its standard input and output: newline-delimited JSON-RPC
// 2.0, the Model Context Protocol the `sicdm-mcp` server speaks. The program sends requests,
// and SI CDM answers them against the open document:
//
//   tools/list                       the tools (the same as for AI agents)
//   tools/call {name, arguments}     run one: create_solid, run_command, get_scene, cam, ...
//   sicdm/say {text}                 a line in the command's output (a request or a notification)
//
// Any other line the program writes is shown as output too. The command succeeds when the
// program exits with 0; otherwise what it wrote to its standard error is shown. A program
// that takes longer than its manifest's "timeout" (seconds, default 120) is stopped.
//
// plugin.json:
//
//   {
//     "name": "gears",
//     "version": "1.0",
//     "description": "Spur gears from a module and a tooth count",
//     "author": "...",
//     "commands": [
//       {"name": "SPURGEAR", "usage": "SPURGEAR teeth module thickness",
//        "help": "A spur gear, extruded", "run": ["python3", "gears.py"],
//        "run_windows": ["python", "gears.py"]}
//     ],
//     "menu": [{"label": "Spur Gear (20 teeth, module 2)", "command": "SPURGEAR 20 2 8"}],
//     "timeout": 120
//   }
//
// "run" is the program and its first arguments; the command's own arguments follow. A first
// word that names a file in the plugin's folder is run from there; otherwise it is looked for
// on the PATH. The program gets SICDM_PLUGIN_DIR (its folder) and SICDM_VERSION.
//
// Plugins are found in each folder of $SICDM_PLUGIN_PATH (separated by ':' or, on Windows,
// ';') and in the per-user plugins folder (pluginsFolder()). A plugin's command can't take the
// name of a built-in command.

#include <string>
#include <vector>

namespace sicdm {

struct CommandOutput;
struct Document;
class ViewHost;

struct PluginCommand {
    std::string name; // upper case
    std::string usage, help;
    std::vector<std::string> run;
};

struct PluginMenuItem {
    std::string label, command;
};

struct Plugin {
    std::string name, version, description, author;
    std::string dir;
    std::vector<PluginCommand> commands;
    std::vector<PluginMenuItem> menu;
    double timeout = 120.0;
    std::string error; // why it couldn't be loaded (then it has no commands)
};

// %APPDATA%\SI-CDM\Plugins, ~/Library/Application Support/SI-CDM/Plugins,
// $XDG_CONFIG_HOME/sicdm/plugins (~/.config/sicdm/plugins).
std::string pluginsFolder();

// Reads a manifest. False (and Plugin::error) when it can't be used.
bool loadPlugin(const std::string& dir, Plugin& out);

// The plugins found (read once; reloadPlugins() reads them again).
const std::vector<Plugin>& plugins();
void reloadPlugins();

// A plugin command by name (any case); null if none.
const PluginCommand* findPluginCommand(const std::string& name, const Plugin** owner = nullptr);

// Runs a plugin command against the document.
CommandOutput runPluginCommand(Document& doc, ViewHost* view, const Plugin& plugin, const PluginCommand& command, const std::vector<std::string>& args);

} // namespace sicdm
