#include "TestFramework.hpp"

#include "core/Command.hpp"

#include <set>
#include <string>

using namespace sicdm;

TEST_CASE("command names and aliases resolve case-insensitively") {
    CHECK(findCommand("circle") && std::string(findCommand("circle")->name) == "CIRCLE");
    CHECK(findCommand("C") && std::string(findCommand("C")->name) == "CIRCLE");
    CHECK(findCommand("rectang") && std::string(findCommand("rectang")->name) == "RECT");
    CHECK(findCommand("  pl ") && std::string(findCommand("pl")->name) == "PLINE");
    CHECK(findCommand("nope") == nullptr);
    CHECK(findCommand("") == nullptr);
}

TEST_CASE("aliases are unique across commands") {
    std::set<std::string> seen;
    for (const CommandSpec& c : commandTable()) {
        CHECK(seen.insert(c.name).second);
        std::string a = c.aliases;
        std::size_t start = 0;
        while (start < a.size()) {
            const std::size_t sp = a.find(' ', start);
            const std::string word = a.substr(start, sp - start);
            if (!word.empty()) CHECK(seen.insert(word).second);
            if (sp == std::string::npos) break;
            start = sp + 1;
        }
    }
}

TEST_CASE("completion matches names and aliases by prefix") {
    const auto m = completeCommand("cy");
    CHECK(m.size() == 1 && std::string(m[0]->name) == "CYLINDER");
    CHECK(completeCommand("").empty());
    CHECK(completeCommand("p").size() >= 4); // PLINE POINT PYRAMID PRISM PLANE PERSP PANEL
}

TEST_CASE("tokenizer upper-cases the name and keeps args") {
    const ParsedCommand pc = tokenizeCommand("  circle   10,20  5 ");
    CHECK(pc.name == "CIRCLE");
    CHECK(pc.args.size() == 2);
    CHECK(pc.args[0] == "10,20");
    CHECK(pc.args[1] == "5");
    CHECK(tokenizeCommand("   ").name.empty());
    const ParsedCommand q = tokenizeCommand(R"(import "my drawings/plan A.png" @10,20)");
    CHECK(q.args.size() == 2 && q.args[0] == "my drawings/plan A.png");
    CHECK(q.quoted.size() == 2 && q.quoted[0] && !q.quoted[1]);
    // \" inside quotes is a quote of the name.
    const ParsedCommand e = tokenizeCommand(R"(select "the \"big\" one" x)");
    CHECK(e.args.size() == 2 && e.args[0] == "the \"big\" one");
}

TEST_CASE("point input: absolute, relative and polar") {
    auto p = parsePointInput("10,-5.5");
    CHECK(p && !p->relative && !p->polar);
    CHECK(p && p->resolve({100, 100}).x == 10.0f && p->resolve({100, 100}).y == -5.5f);

    p = parsePointInput("@3,4");
    CHECK(p && p->relative);
    CHECK_NEAR(p->resolve({1, 1}).x, 4, 1e-6);
    CHECK_NEAR(p->resolve({1, 1}).y, 5, 1e-6);

    p = parsePointInput("@10<90");
    CHECK(p && p->relative && p->polar);
    CHECK_NEAR(p->resolve({2, 3}).x, 2, 1e-5);
    CHECK_NEAR(p->resolve({2, 3}).y, 13, 1e-5);

    p = parsePointInput("5<0");
    CHECK(p && p->polar && !p->relative);
    CHECK_NEAR(p->resolve({9, 9}).x, 5, 1e-6);

    CHECK(!parsePointInput("abc"));
    CHECK(!parsePointInput("1,2,3"));
    CHECK(!parsePointInput("1,"));
    CHECK(!parsePointInput("12"));
}

TEST_CASE("vectors and numbers parse strictly") {
    auto v = parseVec3("@1,2,3");
    CHECK(v && v->x == 1.0f && v->y == 2.0f && v->z == 3.0f);
    v = parseVec3("4,5");
    CHECK(v && v->z == 0.0f);
    CHECK(!parseVec3("1"));
    CHECK(!parseVec3("1,2,3,4"));
    CHECK(parseNumber("+2.5") && *parseNumber("+2.5") == 2.5f);
    CHECK(!parseNumber("2.5mm"));
    CHECK(!parseNumber(""));
}
