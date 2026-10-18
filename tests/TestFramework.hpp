#pragma once

// Minimal dependency-free test harness.

#include <cmath>
#include <cstdio>
#include <functional>
#include <vector>

namespace sicdm::test {

struct Case {
    const char* name;
    std::function<void()> fn;
};

inline std::vector<Case>& registry() {
    static std::vector<Case> cases;
    return cases;
}

inline int& failures() {
    static int n = 0;
    return n;
}

struct Registrar {
    Registrar(const char* name, std::function<void()> fn) { registry().push_back({name, std::move(fn)}); }
};

} // namespace sicdm::test

#define SICDM_CONCAT_(a, b) a##b
#define SICDM_CONCAT(a, b) SICDM_CONCAT_(a, b)

#define TEST_CASE(name)                                                                          \
    static void SICDM_CONCAT(test_fn_, __LINE__)();                                              \
    static ::sicdm::test::Registrar SICDM_CONCAT(test_reg_, __LINE__)(name, SICDM_CONCAT(test_fn_, __LINE__)); \
    static void SICDM_CONCAT(test_fn_, __LINE__)()

#define CHECK(expr)                                                                \
    do {                                                                           \
        if (!(expr)) {                                                             \
            std::fprintf(stderr, "  %s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #expr); \
            ++::sicdm::test::failures();                                           \
        }                                                                          \
    } while (0)

#define CHECK_NEAR(a, b, eps)                                                                   \
    do {                                                                                        \
        const double va_ = (a), vb_ = (b);                                                      \
        if (std::fabs(va_ - vb_) > (eps)) {                                                     \
            std::fprintf(stderr, "  %s:%d: CHECK_NEAR(%s, %s) failed: %g vs %g\n", __FILE__, __LINE__, \
                         #a, #b, va_, vb_);                                                     \
            ++::sicdm::test::failures();                                                        \
        }                                                                                       \
    } while (0)
