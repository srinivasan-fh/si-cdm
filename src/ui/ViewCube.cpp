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
constexpr float kBorder = 0.30f;   // width of edge/corner strips in face units (face spans -1..1)
constexpr float kDragPx = 3.0f;

struct Face {
    Vec3 normal;
    Vec3 s, t; // label axes: s to the right, t up when reading the label
    const char* label;
};

// Label axes chosen so every label reads upright from its standard view.
constexpr std::array<Face, 6> kFaces = {{
    {{0, 0, 1}, {1, 0, 0}, {0, 1, 0}, "TOP"},
    {{0, 0, -1}, {1, 0, 0}, {0, -1, 0}, "BOTTOM"},
    {{0, -1, 0}, {1, 0, 0}, {0, 0, 1}, "FRONT"},
    {{0, 1, 0}, {-1, 0, 0}, {0, 0, 1}, "BACK"},
    {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, "RIGHT"},
    {{-1, 0, 0}, {0, -1, 0}, {0, 0, 1}, "LEFT"},
}};

bool sameDir(const Vec3& a, const Vec3& b) { return a.x == b.x && a.y == b.y && a.z == b.z; }

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

bool insideQuad(const ImVec2 (&q)[4], ImVec2 p) {
    int sign = 0;
    for (int i = 0; i < 4; ++i) {
        const ImVec2 a = q[i], b = q[(i + 1) % 4];
        const float c = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
        const int sg = c > 0 ? 1 : (c < 0 ? -1 : 0);
        if (sg == 0) continue;
        if (sign == 0) sign = sg;
        else if (sg != sign) return false;
    }
    return true;
}

ImU32 grey(float v, float a = 1.0f) {
    const int c = static_cast<int>(std::clamp(v, 0.0f, 1.0f) * 255.0f);
    return IM_COL32(c, c, c + 6 > 255 ? 255 : c + 6, static_cast<int>(a * 255.0f));
}

// Draws `text` onto a cube face as projected glyph quads, so labels follow the face.
void drawFaceLabel(ImDrawList* dl, const char* text, const Face& f, float alpha,
                   const std::function<ImVec2(const Vec3&)>& project) {
    ImFont* font = ImGui::GetFont();
    float width = 0.0f;
    for (const char* c = text; *c; ++c)
        if (const ImFontGlyph* g = font->FindGlyph(static_cast<ImWchar>(*c))) width += g->AdvanceX;
    // Fit the word into ~70% of the face width, capped so short words stay modest.
    const float scale = std::min(1.4f / width, 0.62f / font->FontSize);
    float x = -width * 0.5f;
    const float yTop = font->FontSize * 0.5f; // vertically centred
    const ImTextureID tex = font->ContainerAtlas->TexID;
    const ImU32 col = IM_COL32(70, 74, 82, static_cast<int>(alpha * 255.0f));
    const Vec3 base = f.normal * 1.002f;
    for (const char* c = text; *c; ++c) {
        const ImFontGlyph* g = font->FindGlyph(static_cast<ImWchar>(*c));
        if (!g) continue;
        auto at = [&](float gx, float gy) {
            return project(base + f.s * ((x + gx) * scale) + f.t * ((yTop - gy) * scale));
        };
        if (g->Visible)
            dl->AddImageQuad(tex, at(g->X0, g->Y0), at(g->X1, g->Y0), at(g->X1, g->Y1), at(g->X0, g->Y1),
                             {g->U0, g->V0}, {g->U1, g->V0}, {g->U1, g->V1}, {g->U0, g->V1}, col);
        x += g->AdvanceX;
    }
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

    // Rotation-only projection: the cube turns with the camera but never moves.
    const Mat4 view = cam.view();
    auto project = [&](const Vec3& p) {
        const Vec3 e = view.transformDir(p);
        return ImVec2{center.x + e.x * half, center.y - e.y * half};
    };

    // Hit area for the cube itself.
    ImGui::SetCursorScreenPos({center.x - kCubePx * 0.5f, center.y - kCubePx * 0.5f});
    ImGui::InvisibleButton("cube", {kCubePx, kCubePx}, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
    const bool cubeHovered = ImGui::IsItemHovered();
    const bool cubeActive = ImGui::IsItemActive();
    const ImVec2 mouse = ImGui::GetIO().MousePos;

    // Find the hovered region among front-facing cells (they never overlap).
    std::optional<Vec3> hoverDir;
    struct Cell {
        ImVec2 q[4];
        Vec3 dir;
    };
    std::array<Cell, 54> cells{};
    std::array<float, 54> cellFacing{};
    int cellCount = 0;
    const float edges[4] = {-1.0f, -1.0f + kBorder, 1.0f - kBorder, 1.0f};
    for (const Face& f : kFaces) {
        const float facing = view.transformDir(f.normal).z;
        if (facing <= 0.02f) continue;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) {
                Cell& c = cells[static_cast<std::size_t>(cellCount)];
                cellFacing[static_cast<std::size_t>(cellCount)] = facing;
                ++cellCount;
                const float s0 = edges[i], s1 = edges[i + 1], t0 = edges[j], t1 = edges[j + 1];
                c.q[0] = project(f.normal + f.s * s0 + f.t * t0);
                c.q[1] = project(f.normal + f.s * s1 + f.t * t0);
                c.q[2] = project(f.normal + f.s * s1 + f.t * t1);
                c.q[3] = project(f.normal + f.s * s0 + f.t * t1);
                c.dir = f.normal + f.s * static_cast<float>(i - 1) + f.t * static_cast<float>(j - 1);
                if (cubeHovered && !m_dragging && insideQuad(c.q, mouse)) hoverDir = c.dir;
            }
    }

    // Soft shadow + faces.
    dl->AddCircleFilled({center.x + 2, center.y + 4}, half * 1.75f, IM_COL32(0, 0, 0, 50), 40);
    for (int k = 0; k < cellCount; ++k) {
        const Cell& c = cells[static_cast<std::size_t>(k)];
        const float shade = 0.50f + 0.36f * cellFacing[static_cast<std::size_t>(k)];
        const bool hot = hoverDir && sameDir(*hoverDir, c.dir);
        const ImU32 fill = hot ? IM_COL32(74, 140, 230, 255) : grey(shade);
        dl->AddQuadFilled(c.q[0], c.q[1], c.q[2], c.q[3], fill);
    }
    for (const Face& f : kFaces) {
        const float facing = view.transformDir(f.normal).z;
        if (facing <= 0.02f) continue;
        const ImVec2 a = project(f.normal - f.s - f.t), b = project(f.normal + f.s - f.t);
        const ImVec2 c = project(f.normal + f.s + f.t), d = project(f.normal - f.s + f.t);
        dl->AddQuad(a, b, c, d, IM_COL32(60, 64, 72, 255), 1.2f);
        drawFaceLabel(dl, f.label, f, std::clamp(facing * 2.0f - 0.2f, 0.0f, 1.0f), project);
    }
    // Orange outline around every cell of the hovered region (edges span 2 faces, corners 3).
    if (hoverDir)
        for (int k = 0; k < cellCount; ++k) {
            const Cell& c = cells[static_cast<std::size_t>(k)];
            if (sameDir(*hoverDir, c.dir)) dl->AddQuad(c.q[0], c.q[1], c.q[2], c.q[3], IM_COL32(255, 170, 40, 255), 2.0f);
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
