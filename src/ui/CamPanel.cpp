// The dock's CAM page: the milling job's setup, its operations (a list to pick, reorder,
// switch off, copy and delete from) and the fields of the picked one, then the same for the
// lathe. Every change goes through a command (CAM EDIT, LATHE MOVE ...), so it is undone,
// logged and scripted like one typed.

#include "core/CamEdit.hpp"
#include "ui/LandingUI.hpp"
#include "ui/Theme.hpp"

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
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
        if (ImGui::Button("Set Up the Selected Part", {-1, 0})) cmd.commands.push_back("CAM SETUP");
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
        if (ImGui::Button("Set Up the Selected Part##lathe", {-1, 0})) cmd.commands.push_back("LATHE SETUP");
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
            for (const char* t : {"LATHE TOOL 1 TURN NOSE 0.8 INSERT C", "LATHE TOOL 2 TURN NOSE 0.4 INSERT D", "LATHE TOOL 3 GROOVE 3",
                                  "LATHE TOOL 4 DRILL 8", "LATHE TOOL 5 BORE 6 NOSE 0.2", "LATHE TOOL 6 THREAD"})
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
                                    {"3D Rough (leave 0.3 mm)", "CAM ROUGH LEAVE 0.3"},
                                    {"Adaptive Rough (trochoidal)", "CAM ROUGH ADAPTIVE LEAVE 0.3"},
                                    {"Rest Rough (what the tools before left)", "CAM ROUGH REST LEAVE 0.3"},
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
                                    {"Rough Turn (leave 0.2 mm)", "LATHE ROUGH LEAVE 0.2"},
                                    {"Finish Turn", "LATHE FINISH"},
                                    {"Groove the Recesses", "LATHE GROOVE"},
                                    {"Thread (M, pitch 1.5)", "LATHE THREAD PITCH 1.5"},
                                    {nullptr, nullptr},
                                    {"Drill the Bore", "LATHE DRILL"},
                                    {"Rough Bore (leave 0.1 mm)", "LATHE BORE LEAVE 0.1"},
                                    {"Finish the Bore", "LATHE FINISH INSIDE"},
                                    {"Groove the Bore", "LATHE GROOVE INSIDE"},
                                    {"Thread the Bore (pitch 1.5)", "LATHE THREAD PITCH 1.5 INSIDE"},
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

} // namespace sicdm
