// The dock's CAM page: the milling job's setup, its operations (a list to pick, reorder,
// switch off, copy and delete from) and the fields of the picked one, then the same for the
// lathe. Every change goes through a command (CAM EDIT, LATHE MOVE ...), so it is undone,
// logged and scripted like one typed.

#include "core/CamEdit.hpp"
#include "core/CamLibrary.hpp"
#include "core/Preferences.hpp"
#include "ui/LandingUI.hpp"
#include "ui/Theme.hpp"

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <limits>
#include <string>

namespace sicdm {

namespace {

std::string quotedName(const std::string& s) { return "\"" + s + "\""; }

const char* camKindLabel(CamOpKind k) {
    switch (k) {
    case CamOpKind::Face: return "Face";
    case CamOpKind::Contour: return "Contour";
    case CamOpKind::Pocket: return "Pocket";
    case CamOpKind::Rough: return "Rough";
    case CamOpKind::Parallel: return "Parallel finish";
    case CamOpKind::Waterline: return "Waterline finish";
    case CamOpKind::Drill: return "Drill";
    case CamOpKind::Bore: return "Helical bore";
    case CamOpKind::Rotary: return "Rotary (4-axis)";
    case CamOpKind::Surface5: return "5-axis finish";
    case CamOpKind::Swarf: return "Swarf (5-axis flank)";
    }
    return "Operation";
}

std::string latheKindLabel(const LatheOperation& op) {
    const char* in = op.inside ? " inside" : "";
    switch (op.kind) {
    case LatheOpKind::Face: return "Face";
    case LatheOpKind::Rough: return op.inside ? "Rough bore" : "Rough turn";
    case LatheOpKind::Finish: return std::string("Finish") + in;
    case LatheOpKind::Groove: return std::string("Groove") + in;
    case LatheOpKind::Drill: return "Drill";
    case LatheOpKind::Thread: return std::string("Thread") + in;
    case LatheOpKind::PartOff: return "Part off";
    }
    return "Operation";
}

// A number field; true when it changed. `lo`: the smallest value it takes.
bool numberField(const char* label, double& v, const char* format = "%.4g", double lo = -1e9) {
    const bool changed = ImGui::InputDouble(label, &v, 0.0, 0.0, format);
    if (changed) v = std::max(v, lo);
    return changed;
}

// A field with an Auto box: auto is `autoValue` (or NaN, `nanAuto`); unticking starts from
// `start`.
bool autoField(const char* label, double& v, double autoValue, double start, bool nanAuto = false, const char* format = "%.4g") {
    ImGui::PushID(label);
    bool isAuto = nanAuto ? std::isnan(v) : v == autoValue;
    bool changed = false;
    if (ImGui::Checkbox("##auto", &isAuto)) {
        v = isAuto ? (nanAuto ? std::numeric_limits<double>::quiet_NaN() : autoValue) : start;
        changed = true;
    }
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Automatic (from the part, the tool and the material)");
    ImGui::SameLine(0, 4);
    if (isAuto) {
        ImGui::BeginDisabled();
        double shown = start;
        ImGui::InputDouble(label, &shown, 0.0, 0.0, "auto");
        ImGui::EndDisabled();
    } else {
        changed |= ImGui::InputDouble(label, &v, 0.0, 0.0, format);
    }
    ImGui::PopID();
    return changed;
}

// The rows of a list box `rows` high (at most `most`).
float listHeight(std::size_t rows, std::size_t most) {
    return ImGui::GetTextLineHeightWithSpacing() * static_cast<float>(std::clamp<std::size_t>(rows, 2, most)) + ImGui::GetStyle().FramePadding.y * 4;
}

} // namespace

void LandingUI::drawCamPanel(UiCommands& cmd, const Scene& scene, UiState& st) {
    if (!st.cam || !st.lathe) return;
    const CamJob& job = *st.cam;
    const std::string selectedName =
        st.selectedBody >= 0 && st.selectedBody < static_cast<int>(scene.bodies().size()) ? scene.bodies()[static_cast<std::size_t>(st.selectedBody)].name : "";
    ImGui::BeginChild("##campage", {0, 0}, ImGuiChildFlags_None);
    ImGui::PushItemWidth(-90);

    // Milling: the setup.
    ImGui::SeparatorText("Milling");
    if (!job.setup.hasStock) {
        ImGui::TextWrapped("Pick the part to machine (in the 3D view or on the Model page), then set it up. The stock is the part's box plus 2 mm.");
        ImGui::BeginDisabled(selectedName.empty());
        if (ImGui::Button("Set Up the Selected Part", {-1, 0})) {
            // Preferences: the material, post and machine a set-up starts with.
            const Preferences& p = preferences();
            cmd.commands.push_back("CAM SETUP" + (p.camMaterial.empty() ? std::string() : " MATERIAL " + p.camMaterial) +
                                   (p.camPost.empty() ? std::string() : " POST " + p.camPost));
            if (!p.camMachine.empty()) cmd.commands.push_back("CAM MACHINE PRESET " + p.camMachine);
        }
        ImGui::EndDisabled();
    } else {
        const CamSetup& su = job.setup;
        const Vec3 s = su.stockMax - su.stockMin;
        ImGui::TextUnformatted(("Part: " + su.body).c_str());
        ImGui::TextColored(theme::vec(theme::kTextDim), "Stock %.4g x %.4g x %.4g mm", s.x, s.y, s.z);
        const std::string setup = "CAM SETUP " + quotedName(su.body);
        const CamMaterial* mat = camMaterial(su.material);
        if (ImGui::BeginCombo("Material", mat ? mat->name.c_str() : su.material.c_str())) {
            for (const CamMaterial& m : camMaterials())
                if (ImGui::Selectable(m.name.c_str(), mat && m.key == mat->key)) cmd.commands.push_back(setup + " MATERIAL " + m.key);
            ImGui::EndCombo();
        }
        if (ImGui::BeginCombo("Post", su.post.c_str())) {
            for (const std::string& p : camPosts())
                if (ImGui::Selectable(p.c_str(), p == su.post)) cmd.commands.push_back(setup + " POST " + p);
            ImGui::EndCombo();
        }
        if (ImGui::Button(("Machine: " + su.machine.name + "...").c_str(), {-1, 0})) st.showCamMachine = true;
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", su.machine.describe().c_str());
        if (ImGui::Button(("Tools (" + std::to_string(job.tools.size()) + ")...").c_str(), {-1, 0})) st.showCamTools = true;
        if (!selectedName.empty() && selectedName != su.body && ImGui::Button(("Set Up " + selectedName + " Instead").c_str(), {-1, 0}))
            cmd.commands.push_back("CAM SETUP " + quotedName(selectedName));
    }
    drawCamOperations(cmd, st);

    // Turning.
    ImGui::Spacing();
    ImGui::SeparatorText("Turning (lathe)");
    const LatheJob& lj = *st.lathe;
    if (lj.setup.body.empty()) {
        ImGui::TextWrapped("Pick a round part, then set it up for the lathe. It turns about its longest side.");
        ImGui::BeginDisabled(selectedName.empty());
        if (ImGui::Button("Set Up the Selected Part##lathe", {-1, 0})) {
            const Preferences& p = preferences();
            const bool post = std::find(lathePosts().begin(), lathePosts().end(), p.camPost) != lathePosts().end();
            cmd.commands.push_back("LATHE SETUP" + (p.camMaterial.empty() ? std::string() : " MATERIAL " + p.camMaterial) +
                                   (post ? " POST " + p.camPost : std::string()));
        }
        ImGui::EndDisabled();
    } else {
        ImGui::TextUnformatted(("Part: " + lj.setup.body).c_str());
        const std::string setup = "LATHE SETUP " + quotedName(lj.setup.body);
        const CamMaterial* mat = camMaterial(lj.setup.material);
        if (ImGui::BeginCombo("Material##lathe", mat ? mat->name.c_str() : lj.setup.material.c_str())) {
            for (const CamMaterial& m : camMaterials())
                if (ImGui::Selectable(m.name.c_str(), mat && m.key == mat->key)) cmd.commands.push_back(setup + " MATERIAL " + m.key);
            ImGui::EndCombo();
        }
        if (ImGui::BeginCombo("Post##lathe", lj.setup.post.c_str())) {
            for (const char* p : {"fanuc", "haas", "linuxcnc"})
                if (ImGui::Selectable(p, lj.setup.post == p)) cmd.commands.push_back(setup + " POST " + p);
            ImGui::EndCombo();
        }
        if (lj.tools.empty() && ImGui::Button("Add Starter Lathe Tools", {-1, 0})) {
            for (const char* t : {"LATHE TOOL 1 TURN NOSE 0.8mm INSERT C", "LATHE TOOL 2 TURN NOSE 0.4mm INSERT D", "LATHE TOOL 3 GROOVE 3mm",
                                  "LATHE TOOL 4 DRILL 8mm", "LATHE TOOL 5 BORE 6mm NOSE 0.2mm", "LATHE TOOL 6 THREAD"})
                cmd.commands.push_back(t);
            cmd.commands.push_back("LATHE TOOLS");
        }
        drawLatheOperations(cmd, st);
    }
    ImGui::PopItemWidth();
    ImGui::EndChild();
}

void LandingUI::drawCamOperations(UiCommands& cmd, UiState& st) {
    const CamJob& job = *st.cam;
    const int n = static_cast<int>(job.ops.size());
    // A new operation gets picked; one deleted from under the pick moves it.
    if (job.ops.size() > m_camOpsSeen && m_camOpsSeen != 0) st.camSelected = n - 1;
    if (job.ops.size() != m_camOpsSeen && m_camOpsSeen == 0 && n > 0 && st.camSelected < 0) st.camSelected = n - 1;
    m_camOpsSeen = job.ops.size();
    if (st.camSelected >= n) st.camSelected = n - 1;

    ImGui::BeginDisabled(!job.setup.hasStock);
    if (ImGui::Button("+ Add Operation", {-1, 0})) ImGui::OpenPopup("##addcamop");
    ImGui::EndDisabled();
    if (ImGui::BeginPopup("##addcamop")) {
        struct Add {
            const char* label;
            const char* command;
        };
        static const Add kAdds[] = {{"Face", "CAM FACE"},
                                    {"3D Rough (leave 0.3 mm)", "CAM ROUGH LEAVE 0.3mm"},
                                    {"Adaptive Rough (trochoidal)", "CAM ROUGH ADAPTIVE LEAVE 0.3mm"},
                                    {"Rest Rough (what the tools before left)", "CAM ROUGH REST LEAVE 0.3mm"},
                                    {"Pocket", "CAM POCKET"},
                                    {"2D Contour (with tabs)", "CAM CONTOUR TABS 4"},
                                    {"Parallel Finish", "CAM PARALLEL"},
                                    {"Waterline Finish", "CAM WATERLINE"},
                                    {"Spot Drill Holes", "CAM SPOT"},
                                    {"Drill Holes (peck)", "CAM DRILL CYCLE PECK"},
                                    {"Tap Holes", "CAM TAP"},
                                    {"Helical Bore", "CAM BORE"},
                                    {nullptr, nullptr},
                                    {"Rotary Wrap (4-axis)", "CAM ROTARY"},
                                    {"5-axis Surface Finish", "CAM 5AXIS"},
                                    {"Swarf: Walls with the Side (5-axis)", "CAM SWARF"}};
        for (const Add& a : kAdds) {
            if (!a.label) {
                ImGui::Separator();
                continue;
            }
            if (ImGui::MenuItem(a.label)) cmd.commands.push_back(a.command);
        }
        ImGui::EndPopup();
    }
    if (n == 0) {
        ImGui::TextDisabled(job.setup.hasStock ? "No operations yet" : "Set up the part first");
        return;
    }
    if (ImGui::BeginChild("##camops", {0, listHeight(job.ops.size(), 9)}, ImGuiChildFlags_Borders)) {
        for (int i = 0; i < n; ++i) {
            const CamOperation& op = job.ops[static_cast<std::size_t>(i)];
            ImGui::PushID(i);
            bool on = !op.suppressed;
            if (ImGui::Checkbox("##on", &on)) cmd.commands.push_back((on ? "CAM UNSUPPRESS " : "CAM SUPPRESS ") + std::to_string(i + 1));
            if (ImGui::IsItemHovered()) ImGui::SetTooltip(on ? "On: untick to suppress it (kept, not made or posted)" : "Suppressed: tick to use it again");
            ImGui::SameLine(0, 4);
            char label[160];
            std::snprintf(label, sizeof label, "%d. %s  T%d", i + 1, op.name.empty() ? camKindLabel(op.kind) : op.name.c_str(), op.tool);
            if (op.suppressed) ImGui::PushStyleColor(ImGuiCol_Text, theme::vec(theme::kTextFaint));
            if (ImGui::Selectable(label, st.camSelected == i)) st.camSelected = i;
            if (op.suppressed) ImGui::PopStyleColor();
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("%s", op.suppressed ? "Suppressed" : op.generated ? op.summary.c_str() : "Not made yet (Generate, Simulate or Post makes it)");
            if (ImGui::BeginPopupContextItem("##opmenu")) {
                const std::string k = std::to_string(i + 1);
                if (ImGui::MenuItem("Move Up", nullptr, false, i > 0)) cmd.commands.push_back("CAM MOVE " + k + " UP");
                if (ImGui::MenuItem("Move Down", nullptr, false, i + 1 < n)) cmd.commands.push_back("CAM MOVE " + k + " DOWN");
                if (ImGui::MenuItem("Duplicate")) cmd.commands.push_back("CAM DUPLICATE " + k);
                if (ImGui::MenuItem(op.suppressed ? "Unsuppress" : "Suppress")) cmd.commands.push_back((op.suppressed ? "CAM UNSUPPRESS " : "CAM SUPPRESS ") + k);
                ImGui::Separator();
                if (ImGui::MenuItem("Delete")) cmd.commands.push_back("CAM DELETE " + k);
                ImGui::EndPopup();
            }
            ImGui::PopID();
        }
    }
    ImGui::EndChild();
    const int sel = st.camSelected;
    {
        const std::string k = std::to_string(sel + 1);
        const float w = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x * 3) / 4;
        ImGui::BeginDisabled(sel <= 0);
        if (ImGui::Button("Up", {w, 0})) cmd.commands.push_back("CAM MOVE " + k + " UP");
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(sel < 0 || sel + 1 >= n);
        if (ImGui::Button("Down", {w, 0})) cmd.commands.push_back("CAM MOVE " + k + " DOWN");
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(sel < 0);
        if (ImGui::Button("Copy", {w, 0})) cmd.commands.push_back("CAM DUPLICATE " + k);
        ImGui::SameLine();
        if (ImGui::Button("Delete", {w, 0})) cmd.commands.push_back("CAM DELETE " + k);
        ImGui::EndDisabled();
    }

    // The picked operation's fields.
    if (sel >= 0 && sel < n) {
        const CamOperation& op = job.ops[static_cast<std::size_t>(sel)];
        if (sel != m_camEditIndex || !m_camEditDirty) {
            m_camEdit = op;
            m_camEditIndex = sel;
            m_camEditDirty = false;
        }
        CamOperation& e = m_camEdit;
        ImGui::SeparatorText((std::to_string(sel + 1) + ". " + camKindLabel(op.kind)).c_str());
        bool ch = false;
        char name[96];
        std::snprintf(name, sizeof name, "%s", e.name.c_str());
        if (ImGui::InputTextWithHint("Name", camKindLabel(op.kind), name, sizeof name)) e.name = name, ch = true;
        const CamTool* t = job.tool(e.tool);
        if (ImGui::BeginCombo("Tool", t ? t->describe().c_str() : ("T" + std::to_string(e.tool) + " (missing)").c_str())) {
            for (const CamTool& c : job.tools)
                if (ImGui::Selectable(c.describe().c_str(), c.number == e.tool)) e.tool = c.number, ch = true;
            ImGui::EndCombo();
        }
        const CamOpKind k = op.kind;
        const bool drill = k == CamOpKind::Drill, bore = k == CamOpKind::Bore;
        const bool turnsItself = k == CamOpKind::Rotary || k == CamOpKind::Surface5 || k == CamOpKind::Swarf;
        const bool finish = k == CamOpKind::Parallel || k == CamOpKind::Waterline || k == CamOpKind::Rotary || k == CamOpKind::Surface5;
        if (!drill && !bore && k != CamOpKind::Parallel && k != CamOpKind::Waterline && k != CamOpKind::Surface5)
            ch |= autoField("Stepdown mm", e.stepdown, 0.0, 2.0);
        if (k == CamOpKind::Face || k == CamOpKind::Rough || k == CamOpKind::Pocket || finish) ch |= autoField("Stepover mm", e.stepover, 0.0, 1.0);
        if (finish) ch |= numberField("Scallop mm", e.scallop, "%.4g", 0.0005);
        if (!drill) ch |= numberField("Leave mm", e.leave, "%.4g", 0.0);
        if (k != CamOpKind::Rotary) {
            ch |= autoField("Top z", e.top, 0.0, 0.0, true);
            ch |= autoField("Bottom z", e.bottom, 0.0, 0.0, true);
        }
        if (k == CamOpKind::Rough || k == CamOpKind::Pocket) {
            ch |= ImGui::Checkbox("Adaptive (trochoidal)", &e.adaptive);
            ch |= ImGui::Checkbox("Rest (what the tools before left)", &e.rest);
            ch |= ImGui::Checkbox("Zig-zag raster", &e.raster);
        }
        if (k == CamOpKind::Parallel || k == CamOpKind::Surface5 || ((k == CamOpKind::Rough || k == CamOpKind::Pocket) && e.raster))
            ch |= numberField("Angle deg", e.angle);
        if (k == CamOpKind::Parallel || k == CamOpKind::Waterline || k == CamOpKind::Surface5) {
            double lo = e.minSlope, hi = e.maxSlope;
            ch |= numberField("Slope from", lo, "%.3g", 0.0);
            ch |= numberField("Slope to", hi, "%.3g", 0.0);
            e.minSlope = std::clamp(lo, 0.0, 90.0), e.maxSlope = std::clamp(hi, 0.0, 90.0);
        }
        if (k == CamOpKind::Contour || k == CamOpKind::Swarf) {
            int side = e.side;
            if (ImGui::Combo("Walls", &side, "Outside\0Inside\0Both\0")) e.side = side, ch = true;
        }
        if (k == CamOpKind::Contour) {
            int tabs = e.tabs;
            if (ImGui::InputInt("Tabs", &tabs)) e.tabs = std::clamp(tabs, 0, 32), ch = true;
            if (e.tabs > 0) {
                ch |= numberField("Tab width mm", e.tabWidth, "%.4g", 0.1);
                ch |= numberField("Tab height mm", e.tabHeight, "%.4g", 0.1);
            }
            ch |= autoField("Lead arc mm", e.lead, -1.0, 2.0);
        }
        if (k == CamOpKind::Contour || k == CamOpKind::Swarf || k == CamOpKind::Rough || k == CamOpKind::Pocket) {
            bool conv = !e.climb;
            if (ImGui::Checkbox("Conventional (not climb)", &conv)) e.climb = !conv, ch = true;
        }
        if (k == CamOpKind::Rotary) ch |= ImGui::Checkbox("Passes along the axis", &e.along);
        if (k == CamOpKind::Surface5) {
            ch |= numberField("Lead angle deg", e.leadAngle);
            ch |= numberField("Side tilt deg", e.sideTilt);
            ch |= numberField("Max tilt deg", e.maxTilt, "%.4g", 0.0);
        }
        if (drill) {
            int c = static_cast<int>(e.cycle);
            if (ImGui::Combo("Cycle", &c, "Spot\0Drill\0Peck\0Chip break\0Tap\0Bore\0")) e.cycle = static_cast<DrillCycle>(c), ch = true;
            if (e.cycle == DrillCycle::Peck || e.cycle == DrillCycle::ChipBreak) ch |= autoField("Peck mm", e.peck, 0.0, 2.0);
            ch |= numberField("Dwell s", e.dwell, "%.3g", 0.0);
            if (op.autoHoles && !op.generated) ImGui::TextColored(theme::vec(theme::kTextDim), "Holes: found in the part when it is made");
            else ImGui::TextColored(theme::vec(theme::kTextDim), "%zu holes%s", op.holes.size(), op.autoHoles ? " (found in the part)" : " (given)");
            if (!op.autoHoles && ImGui::SmallButton("Find the holes again")) e.autoHoles = true, e.holes.clear(), ch = true;
        }
        if (!turnsItself) {
            ch |= numberField("Table A deg", e.indexA);
            ch |= numberField("Table C deg", e.indexC);
        }
        if (ImGui::TreeNode("Speeds and feeds")) {
            ch |= autoField("RPM", e.setRpm, 0.0, op.rpm > 0 ? op.rpm : 8000.0, false, "%.0f");
            ch |= autoField("Feed mm/min", e.setFeed, 0.0, op.feed > 0 ? op.feed : 1000.0, false, "%.0f");
            ch |= autoField("Plunge mm/min", e.setPlunge, 0.0, op.plunge > 0 ? op.plunge : 300.0, false, "%.0f");
            ImGui::TreePop();
        }
        if (ch) m_camEditDirty = true;
        const std::string args = camEditArgs(op, e);
        ImGui::BeginDisabled(args.empty());
        if (ImGui::Button("Apply", {ImGui::GetContentRegionAvail().x * 0.5f - 4, 0})) {
            cmd.commands.push_back("CAM EDIT " + std::to_string(sel + 1) + args);
            m_camEditDirty = false;
        }
        ImGui::SameLine();
        if (ImGui::Button("Revert", {-1, 0})) m_camEditDirty = false;
        ImGui::EndDisabled();
        if (op.generated && !op.suppressed) ImGui::TextColored(theme::vec(theme::kTextDim), "%s", op.summary.c_str());
    }

    // The whole job.
    ImGui::Spacing();
    const float w = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) / 2;
    if (ImGui::Button("Generate", {w, 0})) cmd.commands.push_back("CAM GENERATE");
    ImGui::SameLine();
    if (ImGui::Button("Simulate", {w, 0})) cmd.commands.push_back("CAM SIMULATE SHOW");
    if (ImGui::Button("Check Program", {w, 0})) cmd.commands.push_back("CAM CHECK");
    ImGui::SameLine();
    if (ImGui::Button("Post G-code...", {w, 0})) requestFile(FilePost, st);
    bool shown = job.showPaths;
    if (ImGui::Checkbox("Show toolpaths", &shown)) cmd.commands.push_back(shown ? "CAM SHOW" : "CAM HIDE");
}

void LandingUI::drawLatheOperations(UiCommands& cmd, UiState& st) {
    const LatheJob& job = *st.lathe;
    const int n = static_cast<int>(job.ops.size());
    if (job.ops.size() > m_latheOpsSeen && m_latheOpsSeen != 0) st.latheSelected = n - 1;
    if (job.ops.size() != m_latheOpsSeen && m_latheOpsSeen == 0 && n > 0 && st.latheSelected < 0) st.latheSelected = n - 1;
    m_latheOpsSeen = job.ops.size();
    if (st.latheSelected >= n) st.latheSelected = n - 1;

    if (ImGui::Button("+ Add Operation##lathe", {-1, 0})) ImGui::OpenPopup("##addlatheop");
    if (ImGui::BeginPopup("##addlatheop")) {
        struct Add {
            const char* label;
            const char* command;
        };
        static const Add kAdds[] = {{"Face", "LATHE FACE"},
                                    {"Rough Turn (leave 0.2 mm)", "LATHE ROUGH LEAVE 0.2mm"},
                                    {"Finish Turn", "LATHE FINISH"},
                                    {"Groove the Recesses", "LATHE GROOVE"},
                                    {"Thread (M, pitch 1.5)", "LATHE THREAD PITCH 1.5mm"},
                                    {nullptr, nullptr},
                                    {"Drill the Bore", "LATHE DRILL"},
                                    {"Rough Bore (leave 0.1 mm)", "LATHE BORE LEAVE 0.1mm"},
                                    {"Finish the Bore", "LATHE FINISH INSIDE"},
                                    {"Groove the Bore", "LATHE GROOVE INSIDE"},
                                    {"Thread the Bore (pitch 1.5)", "LATHE THREAD PITCH 1.5mm INSIDE"},
                                    {nullptr, nullptr},
                                    {"Part Off", "LATHE PARTOFF"}};
        for (const Add& a : kAdds) {
            if (!a.label) {
                ImGui::Separator();
                continue;
            }
            if (ImGui::MenuItem(a.label)) cmd.commands.push_back(a.command);
        }
        ImGui::EndPopup();
    }
    if (n == 0) {
        ImGui::TextDisabled("No lathe operations yet");
        return;
    }
    if (ImGui::BeginChild("##latheops", {0, listHeight(job.ops.size(), 8)}, ImGuiChildFlags_Borders)) {
        for (int i = 0; i < n; ++i) {
            const LatheOperation& op = job.ops[static_cast<std::size_t>(i)];
            ImGui::PushID(1000 + i);
            bool on = !op.suppressed;
            if (ImGui::Checkbox("##on", &on)) cmd.commands.push_back((on ? "LATHE UNSUPPRESS " : "LATHE SUPPRESS ") + std::to_string(i + 1));
            ImGui::SameLine(0, 4);
            char label[160];
            std::snprintf(label, sizeof label, "%d. %s  T%d", i + 1, op.name.empty() ? latheKindLabel(op).c_str() : op.name.c_str(), op.tool);
            if (op.suppressed) ImGui::PushStyleColor(ImGuiCol_Text, theme::vec(theme::kTextFaint));
            if (ImGui::Selectable(label, st.latheSelected == i)) st.latheSelected = i;
            if (op.suppressed) ImGui::PopStyleColor();
            if (ImGui::BeginPopupContextItem("##opmenu")) {
                const std::string k = std::to_string(i + 1);
                if (ImGui::MenuItem("Move Up", nullptr, false, i > 0)) cmd.commands.push_back("LATHE MOVE " + k + " UP");
                if (ImGui::MenuItem("Move Down", nullptr, false, i + 1 < n)) cmd.commands.push_back("LATHE MOVE " + k + " DOWN");
                if (ImGui::MenuItem("Duplicate")) cmd.commands.push_back("LATHE DUPLICATE " + k);
                if (ImGui::MenuItem(op.suppressed ? "Unsuppress" : "Suppress")) cmd.commands.push_back((op.suppressed ? "LATHE UNSUPPRESS " : "LATHE SUPPRESS ") + k);
                ImGui::Separator();
                if (ImGui::MenuItem("Delete")) cmd.commands.push_back("LATHE DELETE " + k);
                ImGui::EndPopup();
            }
            ImGui::PopID();
        }
    }
    ImGui::EndChild();
    const int sel = st.latheSelected;
    {
        const std::string k = std::to_string(sel + 1);
        const float w = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x * 3) / 4;
        ImGui::BeginDisabled(sel <= 0);
        if (ImGui::Button("Up##lathe", {w, 0})) cmd.commands.push_back("LATHE MOVE " + k + " UP");
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(sel < 0 || sel + 1 >= n);
        if (ImGui::Button("Down##lathe", {w, 0})) cmd.commands.push_back("LATHE MOVE " + k + " DOWN");
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(sel < 0);
        if (ImGui::Button("Copy##lathe", {w, 0})) cmd.commands.push_back("LATHE DUPLICATE " + k);
        ImGui::SameLine();
        if (ImGui::Button("Delete##lathe", {w, 0})) cmd.commands.push_back("LATHE DELETE " + k);
        ImGui::EndDisabled();
    }
    if (sel >= 0 && sel < n) {
        const LatheOperation& op = job.ops[static_cast<std::size_t>(sel)];
        if (sel != m_latheEditIndex || !m_latheEditDirty) {
            m_latheEdit = op;
            m_latheEditIndex = sel;
            m_latheEditDirty = false;
        }
        LatheOperation& e = m_latheEdit;
        ImGui::SeparatorText((std::to_string(sel + 1) + ". " + latheKindLabel(op)).c_str());
        bool ch = false;
        char name[96];
        std::snprintf(name, sizeof name, "%s", e.name.c_str());
        if (ImGui::InputTextWithHint("Name##lathe", latheKindLabel(op).c_str(), name, sizeof name)) e.name = name, ch = true;
        const LatheTool* t = job.tool(e.tool);
        if (ImGui::BeginCombo("Tool##lathe", t ? t->describe().c_str() : ("T" + std::to_string(e.tool) + " (missing)").c_str())) {
            for (const LatheTool& c : job.tools)
                if (ImGui::Selectable(c.describe().c_str(), c.number == e.tool)) e.tool = c.number, ch = true;
            ImGui::EndCombo();
        }
        const LatheOpKind k = op.kind;
        if (k == LatheOpKind::Face || k == LatheOpKind::Rough) ch |= autoField("Depth of cut mm", e.stepdown, 0.0, 1.0);
        if (k == LatheOpKind::Rough || k == LatheOpKind::Finish || k == LatheOpKind::Groove) {
            ch |= numberField("Leave mm", e.leave, "%.4g", 0.0);
            ch |= numberField("Leave on faces mm", e.leaveZ, "%.4g", 0.0);
        }
        if (k == LatheOpKind::Rough || k == LatheOpKind::Finish || k == LatheOpKind::Groove || k == LatheOpKind::Thread) {
            ch |= autoField("From z", e.zFrom, 0.0, 0.0, true);
            ch |= autoField("To z", e.zTo, 0.0, -10.0, true);
        }
        if (k == LatheOpKind::Drill) ch |= autoField("Depth mm", e.depth, 0.0, 10.0);
        if (k == LatheOpKind::Drill || k == LatheOpKind::PartOff) ch |= numberField("Peck mm", e.peck, "%.4g", 0.0);
        if (k == LatheOpKind::Thread) {
            ch |= numberField("Pitch mm", e.pitch, "%.4g", 0.05);
            ch |= autoField("Major diameter mm", e.threadDiameter, 0.0, 20.0);
        }
        if (ImGui::TreeNode("Speeds and feeds##lathe")) {
            ch |= autoField("Speed m/min", e.speed, 0.0, op.usedSpeed > 0 ? op.usedSpeed : 200.0, false, "%.0f");
            ch |= autoField("Feed mm/turn", e.feedPerRev, 0.0, op.usedFeed > 0 ? op.usedFeed : 0.2, false, "%.3g");
            ch |= autoField("Fixed RPM", e.rpm, 0.0, op.usedRpm > 0 ? op.usedRpm : 1000.0, false, "%.0f");
            ImGui::TreePop();
        }
        if (ch) m_latheEditDirty = true;
        const std::string args = latheEditArgs(op, e);
        ImGui::BeginDisabled(args.empty());
        if (ImGui::Button("Apply##lathe", {ImGui::GetContentRegionAvail().x * 0.5f - 4, 0})) {
            cmd.commands.push_back("LATHE EDIT " + std::to_string(sel + 1) + args);
            m_latheEditDirty = false;
        }
        ImGui::SameLine();
        if (ImGui::Button("Revert##lathe", {-1, 0})) m_latheEditDirty = false;
        ImGui::EndDisabled();
    }
    ImGui::Spacing();
    const float w = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) / 2;
    if (ImGui::Button("Generate##lathe", {w, 0})) cmd.commands.push_back("LATHE GENERATE");
    ImGui::SameLine();
    if (ImGui::Button("Simulate##lathe", {w, 0})) cmd.commands.push_back("LATHE SIMULATE SHOW");
    ImGui::Checkbox("Canned cycles (G71 / G72)", &m_latheCycles);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Rough turning and boring as G71, facing as G72 (Fanuc, Haas)");
    if (ImGui::Button("Post G-code...##lathe", {-1, 0})) requestFile(FileLathePost, st);
}

void LandingUI::drawCamToolsWindow(UiCommands& cmd, UiState& st) {
    const CamJob& job = *st.cam;
    ImGui::SetNextWindowSize({760, 520}, ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Tools", &st.showCamTools)) {
        ImGui::End();
        return;
    }
    // The job's tools.
    static const char* const kTypes[] = {"Flat end mill", "Ball end mill", "Bull nose", "Face mill", "Drill", "Spot drill", "Chamfer mill", "Tap"};
    const float listW = ImGui::GetContentRegionAvail().x * 0.52f;
    ImGui::BeginChild("##toollist", {listW, 0}, ImGuiChildFlags_None);
    ImGui::TextDisabled("The job's tools (T numbers in the program)");
    if (ImGui::BeginTable("##tools", 4, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_ScrollY,
                          {0, ImGui::GetContentRegionAvail().y - ImGui::GetFrameHeightWithSpacing() * 3})) {
        ImGui::TableSetupColumn("T", ImGuiTableColumnFlags_WidthFixed, 32);
        ImGui::TableSetupColumn("Tool");
        ImGui::TableSetupColumn("Flutes", ImGuiTableColumnFlags_WidthFixed, 48);
        ImGui::TableSetupColumn("Used", ImGuiTableColumnFlags_WidthFixed, 40);
        ImGui::TableHeadersRow();
        for (const CamTool& t : job.tools) {
            int used = 0;
            for (const CamOperation& o : job.ops) used += o.tool == t.number;
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            char label[16];
            std::snprintf(label, sizeof label, "T%d", t.number);
            if (ImGui::Selectable(label, m_toolEditNumber == t.number, ImGuiSelectableFlags_SpanAllColumns)) {
                m_toolEdit = t;
                m_toolEditNumber = t.number;
                m_toolEditDirty = false;
            }
            ImGui::TableNextColumn();
            char what[96];
            std::snprintf(what, sizeof what, "%.4g mm %s%s", t.diameter, kTypes[static_cast<int>(t.type)], t.name.empty() ? "" : ("  " + t.name).c_str());
            ImGui::TextUnformatted(what);
            ImGui::TableNextColumn();
            ImGui::Text("%d", t.flutes);
            ImGui::TableNextColumn();
            if (used) ImGui::Text("%d", used);
            else ImGui::TextDisabled("-");
        }
        ImGui::EndTable();
    }
    if (ImGui::Button("New Tool")) {
        int n = 1;
        while (job.tool(n)) ++n;
        CamTool t;
        t.number = n;
        cmd.commands.push_back(camToolCommand(t));
        m_toolEdit = t;
        m_toolEditNumber = n;
        m_toolEditDirty = false;
    }
    ImGui::SameLine();
    if (ImGui::Button("Load from File...")) requestFile(FileToolsImport, st);
    ImGui::SameLine();
    ImGui::BeginDisabled(job.tools.empty());
    if (ImGui::Button("Save to File...")) requestFile(FileToolsExport, st);
    ImGui::EndDisabled();
    if (job.tools.empty() && ImGui::Button("Add a Starter Set (10 tools)")) {
        for (const char* t : {"CAM TOOL 1 FLAT 6mm", "CAM TOOL 2 FLAT 3mm", "CAM TOOL 3 BALL 6mm", "CAM TOOL 4 BALL 3mm", "CAM TOOL 5 BULL 10mm CORNER 1mm",
                              "CAM TOOL 6 FACE 50mm", "CAM TOOL 7 SPOT 10mm", "CAM TOOL 8 DRILL 5mm", "CAM TOOL 9 DRILL 6mm", "CAM TOOL 10 TAP 6mm"})
            cmd.commands.push_back(t);
    }
    ImGui::EndChild();
    ImGui::SameLine();

    // The picked tool, and the user's library.
    ImGui::BeginChild("##tooledit", {0, 0}, ImGuiChildFlags_None);
    const CamTool* picked = job.tool(m_toolEditNumber);
    if (picked && !m_toolEditDirty) m_toolEdit = *picked;
    if (picked) {
        CamTool& e = m_toolEdit;
        ImGui::PushItemWidth(-110);
        bool ch = false;
        int type = static_cast<int>(e.type);
        if (ImGui::Combo("Type", &type, kTypes, IM_ARRAYSIZE(kTypes))) e.type = static_cast<CamToolType>(type), ch = true;
        ch |= numberField("Diameter mm", e.diameter, "%.4g", 0.05);
        int flutes = e.flutes;
        if (ImGui::InputInt("Flutes", &flutes)) e.flutes = std::clamp(flutes, 1, 20), ch = true;
        ch |= numberField("Flute length mm", e.fluteLength, "%.4g", 0.1);
        if (e.type == CamToolType::BullNose) ch |= numberField("Corner radius mm", e.cornerRadius, "%.4g", 0.0);
        if (e.type == CamToolType::Drill || e.type == CamToolType::Spot || e.type == CamToolType::Chamfer) ch |= numberField("Point angle deg", e.tipAngle, "%.4g", 20.0);
        if (e.type == CamToolType::Tap) ch |= numberField("Pitch mm", e.pitch, "%.4g", 0.05);
        ch |= autoField("Stickout mm", e.stickout, 0.0, e.fluteLength + 5);
        ch |= autoField("Holder dia mm", e.holderDiameter, 0.0, std::max(e.diameter * 2.5, 20.0));
        ch |= numberField("Holder length mm", e.holderLength, "%.4g", 1.0);
        ch |= ImGui::Checkbox("High-speed steel (HSS)", &e.hss);
        ch |= autoField("Speed m/min", e.vc, 0.0, 200.0, false, "%.0f");
        ch |= autoField("Chip load mm", e.fz, 0.0, 0.03, false, "%.3g");
        char text[96];
        std::snprintf(text, sizeof text, "%s", e.name.c_str());
        if (ImGui::InputText("Name", text, sizeof text)) e.name = text, ch = true;
        std::snprintf(text, sizeof text, "%s", e.vendor.c_str());
        if (ImGui::InputText("Vendor", text, sizeof text)) e.vendor = text, ch = true;
        std::snprintf(text, sizeof text, "%s", e.productId.c_str());
        if (ImGui::InputText("Product", text, sizeof text)) e.productId = text, ch = true;
        ImGui::PopItemWidth();
        if (ch) m_toolEditDirty = true;
        ImGui::BeginDisabled(!m_toolEditDirty);
        if (ImGui::Button("Apply")) {
            cmd.commands.push_back(camToolCommand(e));
            m_toolEditDirty = false;
        }
        ImGui::SameLine();
        if (ImGui::Button("Revert")) m_toolEditDirty = false;
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Button("Delete")) {
            cmd.commands.push_back("CAM TOOL DELETE " + std::to_string(e.number));
            m_toolEditNumber = -1;
        }
        ImGui::SameLine();
        if (ImGui::Button("Store in My Library")) {
            cmd.commands.push_back("CAM LIBRARY STORE " + std::to_string(e.number));
            m_userLibraryRead = false;
        }
        ImGui::TextColored(theme::vec(theme::kTextDim), "%s", picked->describe().c_str());
    } else {
        ImGui::TextWrapped("Pick a tool to change it, or add one with New Tool.");
    }

    // The user's own library (shared by all projects).
    ImGui::Spacing();
    ImGui::SeparatorText("My tool library");
    if (!m_userLibraryRead) {
        m_userLibrary.clear();
        m_userLibraryError.clear();
        const std::string path = toolLibraryPath();
        std::error_code ec;
        if (std::filesystem::exists(path, ec) && !loadToolFile(path, m_userLibrary, &m_userLibraryError, nullptr, ToolFileFormat::Native)) m_userLibrary.clear();
        m_userLibraryRead = true;
    }
    if (!m_userLibraryError.empty()) ImGui::TextColored(theme::vec(theme::kError), "%s", m_userLibraryError.c_str());
    if (m_userLibrary.empty()) ImGui::TextDisabled("Empty: store the job's tools, or import a catalogue");
    if (ImGui::BeginChild("##userlib", {0, -ImGui::GetFrameHeightWithSpacing()}, ImGuiChildFlags_Borders)) {
        for (std::size_t i = 0; i < m_userLibrary.size(); ++i) {
            ImGui::PushID(static_cast<int>(i));
            if (ImGui::SmallButton("Use")) cmd.commands.push_back("CAM LIBRARY USE #" + std::to_string(i + 1));
            ImGui::SameLine();
            ImGui::TextUnformatted(m_userLibrary[i].describe().c_str());
            ImGui::PopID();
        }
    }
    ImGui::EndChild();
    ImGui::BeginDisabled(job.tools.empty());
    if (ImGui::Button("Store All the Job's Tools")) {
        cmd.commands.push_back("CAM LIBRARY STORE ALL");
        m_userLibraryRead = false;
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Refresh")) m_userLibraryRead = false;
    ImGui::EndChild();
    ImGui::End();
}

void LandingUI::drawCamMachineWindow(UiCommands& cmd, UiState& st) {
    const CamJob& job = *st.cam;
    ImGui::SetNextWindowSize({520, 600}, ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Machine", &st.showCamMachine)) {
        ImGui::End();
        return;
    }
    if (!m_machineEditDirty) m_machineEdit = job.setup.machine;
    CamMachine& m = m_machineEdit;
    ImGui::PushItemWidth(-160);
    if (ImGui::BeginCombo("Start from a preset", m.name.c_str())) {
        for (const CamMachine& p : camMachinePresets())
            if (ImGui::Selectable(p.name.c_str(), p.name == m.name)) {
                cmd.commands.push_back("CAM MACHINE PRESET " + p.name);
                m_machineEditDirty = false;
            }
        ImGui::EndCombo();
    }
    ImGui::TextColored(theme::vec(theme::kTextDim), "Presets are typical figures: set your machine's own below.");
    bool ch = false;
    char name[64];
    std::snprintf(name, sizeof name, "%s", m.name.c_str());
    if (ImGui::InputText("Name", name, sizeof name)) m.name = name, ch = true;

    ImGui::SeparatorText("Travel (spindle nose, machine coordinates, mm)");
    const char* axes[] = {"X", "Y", "Z"};
    for (int i = 0; i < 3; ++i) {
        ImGui::PushID(i);
        double lo = m.travelMin[i], hi = m.travelMax[i];
        ImGui::SetNextItemWidth(110);
        ch |= ImGui::InputDouble("##lo", &lo, 0, 0, "%.1f");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(110);
        ch |= ImGui::InputDouble("##hi", &hi, 0, 0, "%.1f");
        ImGui::SameLine();
        ImGui::Text("%s  %s", axes[i], hi > lo ? "" : "(not checked)");
        m.travelMin[i] = lo, m.travelMax[i] = hi;
        ImGui::PopID();
    }
    ch |= ImGui::Checkbox("G54 work zero known", &m.hasWorkOffset);
    if (m.hasWorkOffset) {
        float w[3] = {static_cast<float>(m.workOffset[0]), static_cast<float>(m.workOffset[1]), static_cast<float>(m.workOffset[2])};
        if (ImGui::InputFloat3("G54 at (mm)", w, "%.2f")) {
            for (int i = 0; i < 3; ++i) m.workOffset[i] = w[i];
            ch = true;
        }
    }

    ImGui::SeparatorText("Rotary axes");
    int kin = m.headHead() ? 2 : m.headTilts() ? 1 : 0;
    if (ImGui::Combo("Kinematics", &kin, "Table tilts and turns (A, C)\0Head tilts (B), table turns (C)\0Head tilts and swivels (B, C)\0")) {
        m.hasA = kin == 0, m.hasB = kin != 0, m.cHead = kin == 2;
        ch = true;
    }
    auto rotary = [&](const char* label, bool& fitted, double& lo, double& hi) {
        ImGui::PushID(label);
        ch |= ImGui::Checkbox(label, &fitted);
        if (fitted) {
            bool cont = lo <= -99999 && hi >= 99999;
            ImGui::SameLine();
            if (ImGui::Checkbox("continuous", &cont)) {
                if (cont) lo = -99999, hi = 99999;
                else lo = -120, hi = 120;
                ch = true;
            }
            if (!cont) {
                ImGui::SameLine();
                ImGui::SetNextItemWidth(80);
                ch |= ImGui::InputDouble("##lo", &lo, 0, 0, "%.0f");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(80);
                ch |= ImGui::InputDouble("deg##hi", &hi, 0, 0, "%.0f");
            }
        }
        ImGui::PopID();
    };
    if (kin == 0) rotary("A (table, about X)", m.hasA, m.aMin, m.aMax);
    else rotary("B (head, about Y)", m.hasB, m.bMin, m.bMax);
    rotary(kin == 2 ? "C (head, about Z)" : "C (table, about Z)", m.hasC, m.cMin, m.cMax);

    ImGui::SeparatorText("Spindle, feeds and tools");
    ch |= numberField("Max spindle rpm", m.maxRpm, "%.0f", 100);
    ch |= numberField("Max feed mm/min", m.maxFeed, "%.0f", 10);
    ch |= numberField("Rapid mm/min", m.rapid, "%.0f", 10);
    ch |= numberField("Tool change s", m.toolChange, "%.3g", 0);
    int pockets = m.toolPockets;
    if (ImGui::InputInt("Tool pockets (0 any)", &pockets)) m.toolPockets = std::max(0, pockets), ch = true;
    ImGui::PopItemWidth();
    if (ch) m_machineEditDirty = true;
    ImGui::Spacing();
    ImGui::BeginDisabled(!m_machineEditDirty);
    if (ImGui::Button("Apply", {120, 0})) {
        cmd.commands.push_back(camMachineCommand(m));
        m_machineEditDirty = false;
    }
    ImGui::SameLine();
    if (ImGui::Button("Revert", {120, 0})) m_machineEditDirty = false;
    ImGui::EndDisabled();
    ImGui::TextColored(theme::vec(theme::kTextDim), "%s", job.setup.machine.describe().c_str());
    ImGui::End();
}

} // namespace sicdm
