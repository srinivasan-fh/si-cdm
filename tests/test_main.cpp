#include "TestFramework.hpp"

#include <chrono>
#include <cstring>

// Optional argument: run only the cases whose name contains it.
int main(int argc, char** argv) {
    using namespace sicdm::test;
    int failedCases = 0;
    double total = 0;
    std::size_t ran = 0;
    for (const Case& c : registry()) {
        if (argc > 1 && !std::strstr(c.name, argv[1])) continue;
        ++ran;
        const int before = failures();
        const auto t0 = std::chrono::steady_clock::now();
        c.fn();
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        total += ms;
        const bool ok = failures() == before;
        if (!ok) ++failedCases;
        std::printf("[%s] %s (%.0f ms)\n", ok ? " OK " : "FAIL", c.name, ms);
    }
    std::printf("\n%zu cases, %d failed, %.1f s\n", ran, failedCases, total / 1000.0);
    return failedCases == 0 ? 0 : 1;
}
