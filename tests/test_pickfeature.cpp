#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/PickFeature.hpp"

#include <cstdio>
#include <string>
#include <vector>

using namespace sicdm;

#define REQUIRE(c)                  \
    do {                            \
        const bool required_ = (c); \
        CHECK(required_);           \
        if (!required_) return;     \
    } while (0)

namespace {

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }

// Runs the pick tool's command for these curves; true when it made a new body.
bool applyPicks(Document& d, ToolId tool, const std::vector<int>& picks, const PickFeatureOptions& o = {}) {
    const std::string line = pickFeatureCommand(tool, d.scene.sketch(), picks, {}, o);
    if (line.empty()) {
        for (const auto& c : d.scene.sketch().curves()) std::fprintf(stderr, "[%s] ", c.name.c_str());
        return std::fprintf(stderr, "no command\n"), false;
    }
    const std::size_t before = d.scene.bodies().size();
    const CommandOutput out = run(d, line);
    if (!out.ok || d.scene.bodies().size() != before + 1)
        std::fprintf(stderr, "%s\n  %s (%zu bodies)\n", line.c_str(), out.lines.empty() ? "" : out.lines.back().c_str(), d.scene.bodies().size());
    return out.ok && d.scene.bodies().size() == before + 1;
}

} // namespace

TEST_CASE("Pick tools: what each wants, and the command it makes") {
    PickFeatureOptions o;
    CHECK(isPickFeatureTool(ToolId::Revolve) && isPickFeatureTool(ToolId::CoonsPatch) && !isPickFeatureTool(ToolId::Extrude));
    CHECK(pickFeatureSpec(ToolId::Revolve, o).maxPicks == 1);
    o.revolveAxis = 2;
    CHECK(pickFeatureSpec(ToolId::Revolve, o).maxPicks == 2); // and the axis line
    CHECK(pickFeatureSpec(ToolId::Loft, o).maxPicks < 0);      // any number, then Apply
    CHECK(pickFeatureSpec(ToolId::CoonsPatch, o).minPicks == 4);
    CHECK(pickFeatureSpec(ToolId::OffsetSurface, o).body);
    CHECK(pickWantsClosed(ToolId::Sweep, 0) && !pickWantsClosed(ToolId::Sweep, 1));
    CHECK(quoteWord("Line \"3\"") == "\"Line 3\"");

    Document d;
    REQUIRE(run(d, "CIRCLE 0,0 10").ok);
    std::string why;
    CHECK(pickFeatureCommand(ToolId::Sweep, d.scene.sketch(), {0}, {}, o, &why).empty()); // still needs the path
    CHECK(!why.empty());
    CHECK(pickFeatureCommand(ToolId::OffsetSurface, d.scene.sketch(), {}, "Surface 1", o) == "SURFACE OFFSET \"Surface 1\" DISTANCE 20mm");
    // A network: the profiles, then the guides after the split.
    for (const char* l : {"LINE 0,0 10,0", "LINE 0,5 10,5", "LINE 0,0 0,5", "LINE 10,0 10,5"}) REQUIRE(run(d, l).ok);
    PickFeatureOptions g;
    CHECK(pickFeatureCommand(ToolId::GordonSurface, d.scene.sketch(), {1, 2, 3, 4}, {}, g).empty()); // the split isn't set yet
    g.networkSplit = 2;
    const std::string net = pickFeatureCommand(ToolId::GordonSurface, d.scene.sketch(), {1, 2, 3, 4}, {}, g);
    CHECK(net.rfind("SURFACE NETWORK ", 0) == 0 && net.find(" ACROSS ") != std::string::npos);
    CHECK(run(d, net).ok); // four lines meeting at the corners: a flat patch
}

TEST_CASE("Pick tools make solids: revolve, sweep, helix, loft, rib") {
    PickFeatureOptions o;
    // Revolve about the plane's vertical axis, then about a picked line.
    Document r;
    REQUIRE(run(r, "RECT 10,0 20,10").ok);
    CHECK(applyPicks(r, ToolId::Revolve, {0}, o));
    REQUIRE(run(r, "RECT 40,0 50,10").ok);
    REQUIRE(run(r, "LINE 30,-10 30,20").ok);
    o.revolveAxis = 2;
    o.revolveAngle = 180;
    CHECK(applyPicks(r, ToolId::Revolve, {1, 2}, o));
    o = {};

    // Sweep a circle along a line drawn on a plane at right angles to it.
    Document s;
    REQUIRE(run(s, "CIRCLE 0,0 3").ok);
    REQUIRE(run(s, "PLANE FRONT").ok);
    REQUIRE(run(s, "LINE 0,0 0,40").ok);
    CHECK(applyPicks(s, ToolId::Sweep, {0, 1}, o));

    // A coil spring.
    Document h;
    REQUIRE(run(h, "PLANE FRONT").ok);
    REQUIRE(run(h, "CIRCLE 15,0 2").ok);
    o.helixPitch = 8;
    o.helixTurns = 4;
    CHECK(applyPicks(h, ToolId::Helix, {0}, o));

    // Loft between a square and a circle drawn on the same plane: spaced apart.
    Document l;
    REQUIRE(run(l, "RECT -10,-10 10,10").ok);
    REQUIRE(run(l, "CIRCLE 0,0 6").ok);
    CHECK(applyPicks(l, ToolId::Loft, {0, 1}, o));
    CHECK(l.scene.bodies().back().mesh.bounds().max.z - l.scene.bodies().back().mesh.bounds().min.z > 19.0f);

    // A rib in an L.
    Document b;
    REQUIRE(run(b, "BOX 60 40 5 @0,0,0").ok);
    REQUIRE(run(b, "BOX 5 40 40 @-27.5,0,0").ok);
    REQUIRE(run(b, "BOOLEAN \"Block 1\" UNION \"Block 2\"").ok);
    REQUIRE(run(b, "PLANE FRONT").ok);
    REQUIRE(run(b, "LINE -25,25 15,5").ok);
    o.ribThickness = 4;
    const std::size_t before = b.scene.bodies().size();
    const std::string line = pickFeatureCommand(ToolId::Rib, b.scene.sketch(), {static_cast<int>(b.scene.sketch().curves().size()) - 1}, {}, o);
    CHECK(run(b, line).ok);
    CHECK(b.scene.bodies().size() == before); // added to the body
}

TEST_CASE("Pick tools make surfaces: planar, extruded, ruled, B-spline, Coons, offset") {
    PickFeatureOptions o;
    Document d;
    REQUIRE(run(d, "CIRCLE 0,0 10").ok);
    CHECK(applyPicks(d, ToolId::PlaneSurface, {0}, o));
    REQUIRE(run(d, "LINE 30,0 50,0").ok);
    CHECK(applyPicks(d, ToolId::TabulatedCylinder, {1}, o));
    REQUIRE(run(d, "LINE 60,0 80,0").ok);
    REQUIRE(run(d, "LINE 60,20 80,25").ok);
    CHECK(applyPicks(d, ToolId::RuledSurface, {2, 3}, o));
    REQUIRE(run(d, "ARC 100,0 110,5 120,0").ok);
    REQUIRE(run(d, "LINE 100,10 120,10").ok);
    REQUIRE(run(d, "ARC 100,20 110,24 120,20").ok);
    CHECK(applyPicks(d, ToolId::BSplinePatch, {4, 5, 6}, o));
    for (const char* l : {"LINE -20,-60 20,-60", "LINE 20,-60 20,-20", "LINE 20,-20 -20,-20", "LINE -20,-20 -20,-60"}) REQUIRE(run(d, l).ok);
    CHECK(applyPicks(d, ToolId::CoonsPatch, {7, 8, 9, 10}, o));
    const std::string surf = d.scene.bodies().back().name;
    const std::size_t before = d.scene.bodies().size();
    o.surfaceDistance = 2;
    CHECK(run(d, pickFeatureCommand(ToolId::OffsetSurface, d.scene.sketch(), {}, surf, o)).ok);
    CHECK(d.scene.bodies().size() == before + 1);
}
