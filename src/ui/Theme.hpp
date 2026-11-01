#pragma once

#include <imgui.h>

#include <string>

// Look of the workbench: fonts, spacing and colours in one place, modelled on
// the dark themes of professional CAD / DCC tools (AutoCAD, Maya, Photoshop).
namespace sicdm::theme {

enum class Font {
    Body,    // Inter 13: labels, menus, controls
    Strong,  // Inter SemiBold 13: titles, active tab, values that matter
    Small,   // Inter 11.5: hints, units, status bar
    Caption, // Inter SemiBold 10.5: panel section captions (upper case)
    Mono,    // JetBrains Mono 12.5: command line, coordinates, readouts
};

// Loads the embedded fonts and sets the style. `uiScale` enlarges the whole UI
// (HiDPI on Windows / Linux); `framebufferScale` is pixels per window point
// (2 on Retina), used to rasterise text sharply.
void apply(float uiScale, float framebufferScale);

ImFont* font(Font f);

// Layout heights shared by the chrome (window points).
constexpr float kMenuBarHeight = 26.0f;
constexpr float kOptionsBarHeight = 38.0f;
constexpr float kTabBarHeight = 30.0f;
constexpr float kStatusBarHeight = 24.0f;
constexpr float kCommandBarHeight = 32.0f;
constexpr float kPanelPadding = 10.0f;

// Colour tokens.
constexpr ImU32 rgb(int r, int g, int b, int a = 255) {
    return static_cast<ImU32>(r) | (static_cast<ImU32>(g) << 8) | (static_cast<ImU32>(b) << 16) |
           (static_cast<ImU32>(a) << 24);
}
// The palette in use (setPalette changes them; the THEME preference).
inline ImU32 kChrome = rgb(28, 30, 34);      // menu bar, status bar
inline ImU32 kPanel = rgb(37, 39, 44);       // docked panels, options bar
inline ImU32 kPanelRaised = rgb(45, 48, 54); // buttons, headers
inline ImU32 kHover = rgb(56, 60, 68);
inline ImU32 kField = rgb(25, 27, 31);       // inputs
inline ImU32 kBorder = rgb(14, 15, 18);      // panel edges
inline ImU32 kLine = rgb(58, 62, 70);        // dividers
inline ImU32 kText = rgb(220, 223, 228);
inline ImU32 kTextDim = rgb(142, 148, 158);
inline ImU32 kTextFaint = rgb(104, 110, 120);
inline ImU32 kAccent = rgb(47, 123, 224);    // selection, active window, primary action
inline ImU32 kAccentHover = rgb(66, 140, 238);
inline ImU32 kAccentText = rgb(110, 172, 255);
inline ImU32 kWarning = rgb(240, 178, 90);
inline ImU32 kSuccess = rgb(110, 205, 130);
inline ImU32 kError = rgb(240, 110, 105);
inline ImU32 kIcon = rgb(205, 210, 218);        // tool icons
inline ImU32 kIconActive = rgb(255, 255, 255);  // on the accent
inline ImU32 kIconOff = rgb(130, 135, 145);     // a tool not there yet
inline ImU32 kTile = rgb(40, 44, 52);           // icon tiles
inline ImU32 kTileHover = rgb(58, 64, 76);
inline ImU32 kRulerBg = rgb(30, 33, 39);
inline ImU32 kRulerCorner = rgb(24, 26, 31);
inline ImU32 kRulerTick = rgb(120, 126, 138);
inline ImU32 kRulerText = rgb(170, 175, 186);
inline ImU32 kErrorField = rgb(92, 38, 40);     // an input holding a mistake

// "dark" (the default), "light" or "high-contrast"; unknown names: dark. The ImGui style
// colours follow at the next applyColors().
void setPalette(const std::string& name);
const std::string& paletteName();
// The ImGui style's colours from the palette (apply() calls it).
void applyColors();

inline ImVec4 vec(ImU32 c) { return ImGui::ColorConvertU32ToFloat4(c); }

// Upper-case caption with a hairline, used to group panel content.
void caption(const char* text);

// RAII font push.
struct ScopedFont {
    explicit ScopedFont(Font f) { ImGui::PushFont(font(f)); }
    ~ScopedFont() { ImGui::PopFont(); }
    ScopedFont(const ScopedFont&) = delete;
    ScopedFont& operator=(const ScopedFont&) = delete;
};

} // namespace sicdm::theme
