#include "ui/ViewCube.hpp"
#include "core/Translate.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <functional>
#include <cmath>
#include <string>

namespace sicdm {

namespace {

constexpr float kCubePx = 112.0f;  // widget size
constexpr float kDragPx = 3.0f;

struct Face {
    Vec3 normal;
    Vec3 s, t; // label axes (the faces' upright reading, kept for the cube's order)
    const char* label;
};

// The six named views.
constexpr std::array<Face, 6> kFaces = {{
    {{0, 0, 1}, {1, 0, 0}, {0, 1, 0}, "TOP"},
    {{0, 0, -1}, {1, 0, 0}, {0, -1, 0}, "BOTTOM"},
    {{0, -1, 0}, {1, 0, 0}, {0, 0, 1}, "FRONT"},
    {{0, 1, 0}, {-1, 0, 0}, {0, 0, 1}, "BACK"},
    {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, "RIGHT"},
    {{-1, 0, 0}, {0, -1, 0}, {0, 0, 1}, "LEFT"},
}};


std::string regionName(const Vec3& d) {
    std::string n;
    auto add = [&n](const char* w) {
        if (!n.empty()) n += " ";
        n += w;
    };
    if (d.z > 0) add("Top");
    if (d.z < 0) add("Bottom");
    if (d.y < 0) add("Front");
    if (d.y > 0) add("Back");
    if (d.x > 0) add("Right");
    if (d.x < 0) add("Left");
    return n;
}

ImU32 grey(float v, float a = 1.0f) {
    const int c = static_cast<int>(std::clamp(v, 0.0f, 1.0f) * 255.0f);
    return IM_COL32(c, c, c + 6 > 255 ? 255 : c + 6, static_cast<int>(a * 255.0f));
}

bool iconButton(const char* id, ImVec2 pos, float size, const char* tip) {
    ImGui::SetCursorScreenPos(pos);
    ImGui::InvisibleButton(id, {size, size});
    const bool hovered = ImGui::IsItemHovered();
    if (hovered) {
        ImGui::GetWindowDrawList()->AddRectFilled(pos, {pos.x + size, pos.y + size}, IM_COL32(70, 76, 88, 200), 4.0f);
        ImGui::SetTooltip("%s", tip);
    }
    return ImGui::IsItemClicked();
}

} // namespace

ViewCubeResult ViewCube::draw(const OrbitCamera& cam, ImVec2 anchor, bool drawing2D) {
    ViewCubeResult res;
    const float pad = 22.0f; // room for the Home / rotate buttons
    const ImVec2 size{kCubePx + pad * 2.0f, kCubePx + pad * 2.0f};
    ImGui::SetNextWindowPos({anchor.x - size.x, anchor.y - size.y});
    ImGui::SetNextWindowSize(size);
    constexpr ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoBackground |
                                       ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
                                       ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoBringToFrontOnFocus;
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {0, 0});
    ImGui::Begin("##viewcube", nullptr, flags);
    ImGui::PopStyleVar();

    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 origin = ImGui::GetWindowPos();
    const ImVec2 center{origin.x + size.x * 0.5f, origin.y + size.y * 0.5f + 2.0f};
    const float half = kCubePx * 0.33f;

    // Rotation-only: the sphere turns with the camera but never moves.
    const Mat4 view = cam.view();

    // Hit area for the sphere.
    ImGui::SetCursorScreenPos({center.x - kCubePx * 0.5f, center.y - kCubePx * 0.5f});
    ImGui::InvisibleButton("cube", {kCubePx, kCubePx}, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
    const bool cubeHovered = ImGui::IsItemHovered();
    const bool cubeActive = ImGui::IsItemActive();
    const ImVec2 mouse = ImGui::GetIO().MousePos;

    // A sphere: the view direction under the mouse is the sphere's own point there (in the
    // eye's frame, x right, y up, z towards the eye), snapped to the nearest of the 26 views
    // (6 faces, 12 edges, 8 corners of the cube it stands for).
    const float radius = half * 1.62f;
    std::optional<Vec3> hoverDir;
    std::array<Vec3, 26> dirs{};
    {
        int n = 0;
        for (int x = -1; x <= 1; ++x)
            for (int y = -1; y <= 1; ++y)
                for (int z = -1; z <= 1; ++z)
                    if (x || y || z) dirs[static_cast<std::size_t>(n++)] = Vec3{static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
    }
    auto onSphere = [&](const Vec3& d) { // where a direction meets the sphere, and how much it faces us
        const Vec3 e = view.transformDir(normalize(d));
        return std::pair<ImVec2, float>{ImVec2{center.x + e.x * radius, center.y - e.y * radius}, e.z};
    };
    {
        const float dx = (mouse.x - center.x) / radius, dy = -(mouse.y - center.y) / radius;
        const float r2 = dx * dx + dy * dy;
        if (cubeHovered && !m_dragging && r2 <= 1.0f) {
            const Vec3 eye{dx, dy, std::sqrt(std::max(0.0f, 1.0f - r2))};
            float best = -2;
            for (const Vec3& d : dirs) {
                const float c = dot(view.transformDir(normalize(d)), eye);
                if (c > best) best = c, hoverDir = d;
            }
        }
    }

    // Shadow, then the ball: lit from the upper left (rings from the rim in to the highlight).
    dl->AddCircleFilled({center.x + 2, center.y + 5}, radius * 1.03f, IM_COL32(0, 0, 0, 60), 48);
    constexpr int kRings = 28;
    const ImVec2 light{center.x - radius * 0.32f, center.y - radius * 0.38f};
    for (int k = 0; k < kRings; ++k) {
        const float t = static_cast<float>(k) / static_cast<float>(kRings - 1); // 0 rim .. 1 highlight
        const float rr = radius * (1.0f - 0.92f * t);
        const ImVec2 c{center.x + (light.x - center.x) * t, center.y + (light.y - center.y) * t};
        dl->AddCircleFilled(c, rr, grey(0.52f + 0.45f * std::pow(t, 0.8f)), 48);
    }
    // Its three great circles (the planes of the standard views): solid in front, faint behind.
    auto greatCircle = [&](const Vec3& u, const Vec3& v) {
        constexpr int kSeg = 72;
        for (int k = 0; k < kSeg; ++k) {
            const float a0 = 2 * kPi * static_cast<float>(k) / kSeg, a1 = 2 * kPi * static_cast<float>(k + 1) / kSeg;
            const auto [p0, f0] = onSphere(u * std::cos(a0) + v * std::sin(a0));
            const auto [p1, f1] = onSphere(u * std::cos(a1) + v * std::sin(a1));
            const bool front = f0 + f1 > 0;
            if (!front && k % 2) continue; // dashed behind
            dl->AddLine(p0, p1, front ? IM_COL32(70, 76, 88, 200) : IM_COL32(70, 76, 88, 70), front ? 1.2f : 1.0f);
        }
    };
    greatCircle({1, 0, 0}, {0, 1, 0});
    greatCircle({1, 0, 0}, {0, 0, 1});
    greatCircle({0, 1, 0}, {0, 0, 1});
    dl->AddCircle(center, radius, IM_COL32(60, 64, 72, 255), 64, 1.2f);
    // The face names where their directions meet the sphere, upright, fading as they turn away.
    for (const Face& f : kFaces) {
        const auto [p, facing] = onSphere(f.normal);
        if (facing <= 0.15f) continue;
        const float a = std::clamp((facing - 0.15f) * 2.5f, 0.0f, 1.0f);
        ImFont* font = ImGui::GetFont();
        const float fs = ImGui::GetFontSize() * (0.85f + 0.3f * facing);
        const ImVec2 ts = font->CalcTextSizeA(fs, 1e9f, 0, f.label);
        // On the ball, a little in from where the direction meets it, so a name near the rim stays on.
        const ImVec2 q{center.x + (p.x - center.x) * 0.8f, center.y + (p.y - center.y) * 0.8f};
        const ImU32 col = IM_COL32(34, 38, 46, static_cast<int>(a * 255.0f));
        dl->AddText(font, fs, {q.x - ts.x * 0.5f, q.y - ts.y * 0.5f}, col, f.label);
        dl->AddText(font, fs, {q.x - ts.x * 0.5f + 0.6f, q.y - ts.y * 0.5f}, col, f.label); // a touch bolder
    }
    // The view under the mouse: a spot where it meets the sphere.
    if (hoverDir) {
        const auto [p, facing] = onSphere(*hoverDir);
        (void)facing;
        dl->AddCircleFilled(p, 8.0f, IM_COL32(74, 140, 230, 230), 24);
        dl->AddCircle(p, 8.0f, IM_COL32(255, 170, 40, 255), 24, 2.0f);
    }

    // Click selects the view; dragging the cube orbits the camera.
    if (ImGui::IsItemActivated()) {
        m_pressPos = mouse;
        m_dragging = false;
    }
    if (cubeActive && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        if (!m_dragging && std::hypot(mouse.x - m_pressPos.x, mouse.y - m_pressPos.y) > kDragPx) m_dragging = true;
        if (m_dragging && !drawing2D) res.orbitDelta = {ImGui::GetIO().MouseDelta.x, ImGui::GetIO().MouseDelta.y};
    }
    if (ImGui::IsItemDeactivated()) {
        if (!m_dragging && hoverDir && ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
            const Vec3 d = *hoverDir;
            const int axes = (d.x != 0) + (d.y != 0) + (d.z != 0);
            if (drawing2D && axes == 1) {
                res.drawingPlane = d.z != 0 ? SketchPlane::Top : (d.y != 0 ? SketchPlane::Front : SketchPlane::Right);
            } else {
                res.viewDirection = d;
                res.open3D = drawing2D; // edges / corners need the 3D window
            }
        }
        m_dragging = false;
    }
    if (hoverDir && !ImGui::IsPopupOpen("cubemenu")) {
        const int axes = (hoverDir->x != 0) + (hoverDir->y != 0) + (hoverDir->z != 0);
        if (drawing2D && axes == 1)
            ImGui::SetTooltip(tr("%s drawing plane"), regionName(*hoverDir).c_str());
        else
            ImGui::SetTooltip(tr("%s view%s"), regionName(*hoverDir).c_str(), drawing2D ? "  (opens 3D window)" : "");
    }
    if (ImGui::IsItemClicked(ImGuiMouseButton_Right)) ImGui::OpenPopup("cubemenu");

    // Home (top-left) and rotate arrows (top-right).
    const float bs = 20.0f;
    const ImVec2 homePos{origin.x + 2.0f, origin.y + 2.0f};
    if (iconButton("home", homePos, bs, drawing2D ? "Fit drawing" : "Home view (isometric, fit all)")) {
        if (drawing2D) res.fit = true;
        else res.home = true;
    }
    {
        const ImVec2 c{homePos.x + bs * 0.5f, homePos.y + bs * 0.5f + 1};
        const ImU32 ic = IM_COL32(200, 205, 214, 255);
        dl->AddTriangleFilled({c.x - 7, c.y - 1}, {c.x, c.y - 7}, {c.x + 7, c.y - 1}, ic);
        dl->AddRectFilled({c.x - 5, c.y - 1}, {c.x + 5, c.y + 6}, ic);
        dl->AddRectFilled({c.x - 1.5f, c.y + 2}, {c.x + 1.5f, c.y + 6}, IM_COL32(30, 32, 38, 255));
    }
    const ImU32 iconCol = IM_COL32(200, 205, 214, 255);
    const bool shift = ImGui::GetIO().KeyShift;
    if (!drawing2D) {
        // Roll arrows (top-right): spin the picture around the viewing direction.
        // dir = +1 counter-clockwise, -1 clockwise. Shift-click rolls 15 degrees.
        auto rollArrow = [&](const char* id, ImVec2 p, float dir, const char* tip) {
            const bool hit = iconButton(id, p, bs, tip);
            const ImVec2 c{p.x + bs * 0.5f, p.y + bs * 0.5f + 1.0f};
            const float r = 6.0f;
            // Screen angles grow clockwise (y down); sweep ~250 degrees.
            const float a0 = dir > 0 ? -kPi * 0.05f : -kPi * 0.95f;
            const float a1 = a0 - dir * kPi * 1.4f;
            dl->PathArcTo(c, r, a0, a1, 16);
            dl->PathStroke(iconCol, ImDrawFlags_None, 1.8f);
            const ImVec2 tipPt{c.x + r * std::cos(a1), c.y + r * std::sin(a1)};
            const ImVec2 t{dir * std::sin(a1), -dir * std::cos(a1)}; // travel direction at the tip
            const ImVec2 n{std::cos(a1), std::sin(a1)};
            dl->AddTriangleFilled({tipPt.x + t.x * 4.0f, tipPt.y + t.y * 4.0f},
                                  {tipPt.x - t.x * 1.5f + n.x * 3.5f, tipPt.y - t.y * 1.5f + n.y * 3.5f},
                                  {tipPt.x - t.x * 1.5f - n.x * 3.5f, tipPt.y - t.y * 1.5f - n.y * 3.5f}, iconCol);
            return hit;
        };
        const float step = shift ? kPi / 12.0f : kPi * 0.5f;
        if (rollArrow("rollccw", {origin.x + size.x - bs * 2 - 4, origin.y + 2}, 1.0f,
                      "Roll 90° counter-clockwise  (Shift: 15°)"))
            res.roll = step;
        if (rollArrow("rollcw", {origin.x + size.x - bs - 2, origin.y + 2}, -1.0f, "Roll 90° clockwise  (Shift: 15°)"))
            res.roll = -step;
    }

    // Projection badge under the cube, flanked by turn-around-Z arrows.
    {
        char label[32];
        const float rollDeg = std::round(degrees(cam.roll()));
        if (drawing2D) std::snprintf(label, sizeof(label), "PLAN");
        else if (rollDeg != 0.0f)
            std::snprintf(label, sizeof(label), "%s %+.0f°", cam.projection() == Projection::Perspective ? "PERSP" : "ORTHO",
                          rollDeg);
        else std::snprintf(label, sizeof(label), "%s", cam.projection() == Projection::Perspective ? "PERSP" : "ORTHO");
        const ImVec2 ts = ImGui::CalcTextSize(label);
        const ImVec2 p{center.x - ts.x * 0.5f - 4, origin.y + size.y - ts.y - 4};
        if (!drawing2D) { // a cursor move with no item after it would stretch the window (ImGui asserts)
            ImGui::SetCursorScreenPos(p);
            if (ImGui::InvisibleButton("proj", {ts.x + 8, ts.y + 2})) res.toggleProjection = true;
        }
        const bool hov = !drawing2D && ImGui::IsItemHovered();
        if (hov)
            ImGui::SetTooltip(rollDeg != 0.0f ? "Toggle perspective / orthographic  [P]\nView rolled; right-click > Level View"
                                              : "Toggle perspective / orthographic  [P]");
        dl->AddText({p.x + 4, p.y + 1}, hov ? IM_COL32(255, 255, 255, 255) : IM_COL32(150, 156, 166, 255), label);

        if (!drawing2D) {
            auto turnArrow = [&](const char* id, float x, float dir, const char* tip) {
                const float w = 14.0f;
                const ImVec2 bp{x, p.y - 1};
                ImGui::SetCursorScreenPos(bp);
                ImGui::InvisibleButton(id, {w, ts.y + 4});
                const bool h = ImGui::IsItemHovered();
                if (h) ImGui::SetTooltip("%s", tip);
                const ImVec2 c{bp.x + w * 0.5f, bp.y + (ts.y + 4) * 0.5f};
                dl->AddTriangleFilled({c.x + 4 * dir, c.y}, {c.x - 3 * dir, c.y - 4}, {c.x - 3 * dir, c.y + 4},
                                      h ? IM_COL32(255, 255, 255, 255) : iconCol);
                return ImGui::IsItemClicked();
            };
            if (turnArrow("turnl", p.x - 16.0f, -1.0f, "Turn 90° around Z (left)")) res.rotateZ = -kPi * 0.5f;
            if (turnArrow("turnr", p.x + ts.x + 10.0f, 1.0f, "Turn 90° around Z (right)")) res.rotateZ = kPi * 0.5f;
        }
    }

    // Right-click menu: extra views.
    if (ImGui::BeginPopup("cubemenu")) {
        if (ImGui::MenuItem(tr("Home View"))) res.home = true;
        if (ImGui::MenuItem(tr("Zoom to Fit"), "F")) res.fit = true;
        if (!drawing2D) {
            if (ImGui::MenuItem(tr("Orthographic"), "P", cam.projection() == Projection::Orthographic))
                res.toggleProjection = true;
            ImGui::Separator();
            if (ImGui::MenuItem(tr("Roll 90° Counter-clockwise"))) res.roll = kPi * 0.5f;
            if (ImGui::MenuItem(tr("Roll 90° Clockwise"))) res.roll = -kPi * 0.5f;
            if (ImGui::MenuItem(tr("Level View (reset roll)"), nullptr, false, cam.roll() != 0.0f)) res.resetRoll = true;
        }
        ImGui::Separator();
        ImGui::TextDisabled("%s", tr("2D Drawing window"));
        for (SketchPlane p : {SketchPlane::Top, SketchPlane::Front, SketchPlane::Right})
            if (ImGui::MenuItem(planeName(p))) res.drawingPlane = p;
        ImGui::Separator();
        if (ImGui::MenuItem(tr("3D Model window"), "Tab", !drawing2D)) res.open3D = true;
        ImGui::EndPopup();
    }

    ImGui::End();
    return res;
}

} // namespace sicdm
