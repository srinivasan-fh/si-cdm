#include "TestFramework.hpp"

#include "ui/Tools.hpp"

#include <cstring>
#include <set>

using namespace sicdm;

TEST_CASE("tool table is ordered by ToolId") {
    for (int i = 0; i < static_cast<int>(ToolId::Count); ++i)
        CHECK(static_cast<int>(toolInfo(static_cast<ToolId>(i)).id) == i);
}

TEST_CASE("every tool sits in exactly one rail slot of its own category") {
    std::multiset<int> seen;
    for (const ToolGroup& g : toolGroups()) {
        CHECK(!g.tools.empty());
        for (ToolId t : g.tools) {
            seen.insert(static_cast<int>(t));
            CHECK(toolInfo(t).category == g.category);
        }
    }
    for (int i = 0; i < static_cast<int>(ToolId::Count); ++i) CHECK(seen.count(i) == 1);
}

TEST_CASE("shortcuts are unique single letters") {
    std::set<char> keys;
    for (int i = 0; i < static_cast<int>(ToolId::Count); ++i) {
        const char* k = toolInfo(static_cast<ToolId>(i)).shortcut;
        if (!k[0]) continue;
        CHECK(std::strlen(k) == 1 && k[0] >= 'A' && k[0] <= 'Z');
        CHECK(keys.insert(k[0]).second);
    }
    // Keys the viewport already uses.
    for (char reserved : {'F', 'P', 'W', 'G', 'Q', 'M'}) CHECK(keys.count(reserved) == 0);
}

TEST_CASE("solid primitives are available") {
    CHECK(isPrimitiveTool(ToolId::Torus));
    CHECK(!isPrimitiveTool(ToolId::Line));
    CHECK(toolInfo(ToolId::Block).available);
}
