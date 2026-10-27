#pragma once

// Text as outlines: the glyphs of the built-in fonts (core/FontData.hpp) read from their
// TrueType tables and flattened to polygons, for raised and cut lettering on parts.

#include "core/Math.hpp"

#include <string>
#include <vector>

namespace sicdm {

enum class TextFont { Sans, SansBold, Mono };

// Contours of `text` (UTF-8; Latin letters, digits and punctuation the font has): capitals
// `height` mm tall, the baseline along y = 0 from x = 0, lines (on '\n') `height` x 1.6
// apart going down. Outer contours run counter-clockwise, holes clockwise; none repeats its
// first point. `width` gets the longest line's advance. Empty (with a reason) on failure.
std::vector<std::vector<Vec2>> textOutline(const std::string& text, float height, TextFont font = TextFont::SansBold, float* width = nullptr,
                                           std::string* why = nullptr);

// The font for a name: sans, bold, mono (nullopt-like: false when unknown).
bool textFontNamed(const std::string& name, TextFont& font);

} // namespace sicdm
