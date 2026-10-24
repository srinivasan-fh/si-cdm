// A program that installs the crash handler and then dies, for the crash report test:
//   crash_probe <dir> segv|exception

#include "core/CrashReport.hpp"

#include <csignal>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

int main(int argc, char** argv) {
    if (argc < 3) return 2;
#if defined(_MSC_VER)
    _set_abort_behavior(0, _WRITE_ABORT_MSG | _CALL_REPORTFAULT); // no dialog on the test machine
#endif
    sicdm::installCrashHandler("probe", argv[1]);
    sicdm::setCrashContext("PROBE 42");
    if (std::strcmp(argv[2], "exception") == 0) throw std::runtime_error("the probe's exception"); // left to std::terminate
    if (std::strcmp(argv[2], "caught") == 0) { // as main() does in the app
        try {
            throw std::runtime_error("the probe's caught exception");
        } catch (const std::exception& e) {
            sicdm::crashOnUncaught(e.what());
        }
    }
    std::raise(SIGSEGV);
    return 0;
}
