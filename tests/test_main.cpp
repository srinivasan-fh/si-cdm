#include "TestFramework.hpp"

int main() {
    using namespace sicdm::test;
    int failedCases = 0;
    for (const Case& c : registry()) {
        const int before = failures();
        c.fn();
        const bool ok = failures() == before;
        if (!ok) ++failedCases;
        std::printf("[%s] %s\n", ok ? " OK " : "FAIL", c.name);
    }
    std::printf("\n%zu cases, %d failed\n", registry().size(), failedCases);
    return failedCases == 0 ? 0 : 1;
}
