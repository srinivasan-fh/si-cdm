#pragma once

// Files the operating system asks the running app to open: on macOS, a project
// double-clicked in the Finder (or dropped on the Dock icon) arrives as an "open documents"
// event, not as a command-line argument. Elsewhere files come on the command line, and
// these do nothing.

#include <string>
#include <vector>

namespace sicdm {

// Call before glfwInit().
void installOpenFileHandler();
// The files asked for since the last call.
std::vector<std::string> takeOpenFileRequests();

} // namespace sicdm
