#include "core/TextOutline.hpp"

#include "core/FontData.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace sicdm {

namespace {

// A TrueType font's tables, read in place (big-endian; reads past the end give 0).
class TrueType {
public:
    explicit TrueType(const fonts::Blob& b) : m_d(b.data), m_n(b.size) {
        if (m_n < 12) return;
        const int tables = u16(4);
        for (int t = 0; t < tables; ++t) {
            const std::size_t r = 12 + 16 * static_cast<std::size_t>(t);
            char tag[5] = {};
            std::memcpy(tag, m_d + std::min(r, m_n - 4), 4);
            const std::uint32_t off = u32(r + 8);
            if (!std::strcmp(tag, "head")) m_head = off;
            else if (!std::strcmp(tag, "maxp")) m_maxp = off;
            else if (!std::strcmp(tag, "hhea")) m_hhea = off;
            else if (!std::strcmp(tag, "hmtx")) m_hmtx = off;
            else if (!std::strcmp(tag, "loca")) m_loca = off;
            else if (!std::strcmp(tag, "glyf")) m_glyf = off;
            else if (!std::strcmp(tag, "cmap")) m_cmap = off;
        }
        if (!m_head || !m_maxp || !m_hhea || !m_hmtx || !m_loca || !m_glyf || !m_cmap) return;
        m_upem = u16(m_head + 18);
        m_longLoca = i16(m_head + 50) != 0;
        m_glyphs = u16(m_maxp + 4);
        m_hMetrics = u16(m_hhea + 34);
        // The Unicode map: a (3,1) or (0,x) subtable in format 4 or 12.
        const int subs = u16(m_cmap + 2);
        for (int s = 0; s < subs; ++s) {
            const std::size_t r = m_cmap + 4 + 8 * static_cast<std::size_t>(s);
            const int platform = u16(r), encoding = u16(r + 2);
            const std::size_t at = m_cmap + u32(r + 4);
            const int format = u16(at);
            if ((platform == 3 && (encoding == 1 || encoding == 10)) || platform == 0)
                if (format == 4 || format == 12)
                    if (!m_map || format == 12) m_map = at, m_mapFormat = format;
        }
        m_ok = m_upem > 0 && m_glyphs > 0 && m_map;
    }

    bool ok() const { return m_ok; }
    float unitsPerEm() const { return static_cast<float>(m_upem); }

    int glyphOf(std::uint32_t c) const {
        if (m_mapFormat == 12) {
            const std::uint32_t groups = u32(m_map + 12);
            for (std::uint32_t g = 0; g < groups; ++g) {
                const std::size_t r = m_map + 16 + 12 * static_cast<std::size_t>(g);
                if (c >= u32(r) && c <= u32(r + 4)) return static_cast<int>(u32(r + 8) + (c - u32(r)));
            }
            return 0;
        }
        if (c > 0xFFFF) return 0;
        const int segs = u16(m_map + 6) / 2;
        for (int s = 0; s < segs; ++s) {
            const std::size_t endAt = m_map + 14 + 2 * static_cast<std::size_t>(s);
            if (static_cast<std::uint32_t>(u16(endAt)) < c) continue;
            const std::size_t startAt = endAt + 2 + 2 * static_cast<std::size_t>(segs);
            const std::uint32_t start = u16(startAt);
            if (start > c) return 0;
            const std::size_t deltaAt = startAt + 2 * static_cast<std::size_t>(segs), rangeAt = deltaAt + 2 * static_cast<std::size_t>(segs);
            const int delta = i16(deltaAt), range = u16(rangeAt);
            if (!range) return static_cast<int>((c + static_cast<std::uint32_t>(delta)) & 0xFFFF);
            const int g = u16(rangeAt + static_cast<std::size_t>(range) + 2 * (c - start));
            return g ? static_cast<int>((static_cast<std::uint32_t>(g) + static_cast<std::uint32_t>(delta)) & 0xFFFF) : 0;
        }
        return 0;
    }

    float advance(int g) const {
        if (g < 0 || g >= m_glyphs || !m_hMetrics) return 0;
        const int k = std::min(g, m_hMetrics - 1);
        return static_cast<float>(u16(m_hmtx + 4 * static_cast<std::size_t>(k)));
    }

    // The glyph's contours in font units, quadratic curves flattened; transformed by the
    // 2x2 matrix m and offset (dx, dy) (composite glyphs place their parts that way).
    void contours(int g, std::vector<std::vector<Vec2>>& out, const float m[4] = nullptr, float dx = 0, float dy = 0, int depth = 0) const {
        static const float kIdentity[4] = {1, 0, 0, 1};
        if (!m) m = kIdentity;
        if (g < 0 || g >= m_glyphs || depth > 8) return;
        const std::size_t from = m_longLoca ? u32(m_loca + 4 * static_cast<std::size_t>(g)) : 2u * u16(m_loca + 2 * static_cast<std::size_t>(g));
        const std::size_t to = m_longLoca ? u32(m_loca + 4 * static_cast<std::size_t>(g) + 4) : 2u * u16(m_loca + 2 * static_cast<std::size_t>(g) + 2);
        if (to <= from) return; // an empty glyph (a space)
        const std::size_t at = m_glyf + from;
        const int count = i16(at);
        auto place = [&](float x, float y) { return Vec2{m[0] * x + m[2] * y + dx, m[1] * x + m[3] * y + dy}; };
        if (count < 0) { // composite: parts placed by offsets and scales
            std::size_t p = at + 10;
            for (int guard = 0; guard < 64; ++guard) {
                const int flags = u16(p), part = u16(p + 2);
                p += 4;
                float ox = 0, oy = 0;
                if (flags & 1) ox = static_cast<float>(i16(p)), oy = static_cast<float>(i16(p + 2)), p += 4;
                else ox = static_cast<float>(static_cast<std::int8_t>(byte(p))), oy = static_cast<float>(static_cast<std::int8_t>(byte(p + 1))), p += 2;
                if (!(flags & 2)) ox = oy = 0; // matched points: rare, placed at the origin
                float s[4] = {1, 0, 0, 1};
                auto f2 = [&](std::size_t q) { return static_cast<float>(i16(q)) / 16384.0f; };
                if (flags & 8) s[0] = s[3] = f2(p), p += 2;
                else if (flags & 0x40) s[0] = f2(p), s[3] = f2(p + 2), p += 4;
                else if (flags & 0x80) s[0] = f2(p), s[1] = f2(p + 2), s[2] = f2(p + 4), s[3] = f2(p + 6), p += 8;
                const float mm[4] = {m[0] * s[0] + m[2] * s[1], m[1] * s[0] + m[3] * s[1], m[0] * s[2] + m[2] * s[3], m[1] * s[2] + m[3] * s[3]};
                const Vec2 o = place(ox, oy);
                contours(part, out, mm, o.x, o.y, depth + 1);
                if (!(flags & 0x20)) break;
            }
            return;
        }
        std::vector<int> ends(static_cast<std::size_t>(count));
        for (int c = 0; c < count; ++c) ends[static_cast<std::size_t>(c)] = u16(at + 10 + 2 * static_cast<std::size_t>(c));
        if (ends.empty()) return;
        const auto points = static_cast<std::size_t>(ends.back() + 1);
        std::size_t p = at + 10 + 2 * static_cast<std::size_t>(count);
        p += 2 + u16(p); // skip the hinting instructions
        std::vector<std::uint8_t> flags;
        while (flags.size() < points && p < m_n) {
            const std::uint8_t f = byte(p++);
            flags.push_back(f);
            if (f & 8)
                for (int r = byte(p++); r > 0 && flags.size() < points; --r) flags.push_back(f);
        }
        flags.resize(points, 0);
        std::vector<float> xs(points), ys(points);
        for (int axis = 0; axis < 2; ++axis) {
            const std::uint8_t shortBit = axis ? 4 : 2, sameBit = axis ? 32 : 16;
            float v = 0;
            for (std::size_t k = 0; k < points; ++k) {
                const std::uint8_t f = flags[k];
                if (f & shortBit) v += (f & sameBit) ? byte(p) : -static_cast<float>(byte(p)), ++p;
                else if (!(f & sameBit)) v += static_cast<float>(i16(p)), p += 2;
                (axis ? ys : xs)[k] = v;
            }
        }
        std::size_t first = 0;
        for (const int e : ends) {
            const auto last = static_cast<std::size_t>(e);
            if (last < first || last >= points) break;
            const std::size_t n = last - first + 1;
            auto pt = [&](std::size_t k) { return Vec2{xs[first + k % n], ys[first + k % n]}; };
            auto on = [&](std::size_t k) { return (flags[first + k % n] & 1) != 0; };
            // Start on an on-curve point (or between two off-curve ones).
            std::size_t s = 0;
            while (s < n && !on(s)) ++s;
            Vec2 start = s < n ? pt(s) : Vec2{(pt(0).x + pt(n - 1).x) * 0.5f, (pt(0).y + pt(n - 1).y) * 0.5f};
            if (s == n) s = n - 1; // all off-curve: start between the last and the first
            std::vector<Vec2> loop{place(start.x, start.y)};
            Vec2 prev = start;
            for (std::size_t k = 1; k <= n; ++k) {
                const std::size_t i = s + k;
                if (on(i)) {
                    if (k < n) loop.push_back(place(pt(i).x, pt(i).y));
                    prev = pt(i);
                    continue;
                }
                const Vec2 c = pt(i);
                const Vec2 end = on(i + 1) ? pt(i + 1) : Vec2{(c.x + pt(i + 1).x) * 0.5f, (c.y + pt(i + 1).y) * 0.5f};
                const int steps = 6;
                for (int t = 1; t <= steps; ++t) {
                    const float u = static_cast<float>(t) / steps, w = 1 - u;
                    const Vec2 q{w * w * prev.x + 2 * w * u * c.x + u * u * end.x, w * w * prev.y + 2 * w * u * c.y + u * u * end.y};
                    loop.push_back(place(q.x, q.y));
                }
                prev = end;
                if (on(i + 1)) ++k; // its end point is placed already
            }
            // The walk returns to the start: drop the repeat and points on top of each other.
            std::vector<Vec2> clean;
            for (const Vec2& q : loop)
                if (clean.empty() || std::fabs(q.x - clean.back().x) + std::fabs(q.y - clean.back().y) > 1e-3f) clean.push_back(q);
            while (clean.size() > 1 && std::fabs(clean.front().x - clean.back().x) + std::fabs(clean.front().y - clean.back().y) <= 1e-3f) clean.pop_back();
            if (clean.size() >= 3) out.push_back(std::move(clean));
            first = last + 1;
        }
    }

private:
    std::uint8_t byte(std::size_t o) const { return o < m_n ? m_d[o] : 0; }
    int u16(std::size_t o) const { return (byte(o) << 8) | byte(o + 1); }
    int i16(std::size_t o) const { return static_cast<std::int16_t>(static_cast<std::uint16_t>(u16(o))); }
    std::uint32_t u32(std::size_t o) const { return (static_cast<std::uint32_t>(u16(o)) << 16) | static_cast<std::uint32_t>(u16(o + 2)); }

    const unsigned char* m_d;
    std::size_t m_n;
    std::size_t m_head = 0, m_maxp = 0, m_hhea = 0, m_hmtx = 0, m_loca = 0, m_glyf = 0, m_cmap = 0, m_map = 0;
    int m_mapFormat = 0, m_upem = 0, m_glyphs = 0, m_hMetrics = 0;
    bool m_longLoca = false, m_ok = false;
};

float areaOf(const std::vector<Vec2>& p) {
    double a = 0;
    for (std::size_t i = 0, j = p.size() - 1; i < p.size(); j = i++) a += static_cast<double>(p[j].x) * p[i].y - static_cast<double>(p[i].x) * p[j].y;
    return static_cast<float>(a * 0.5);
}

bool inside(const std::vector<Vec2>& poly, Vec2 q) {
    bool in = false;
    for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++)
        if ((poly[i].y > q.y) != (poly[j].y > q.y) && q.x < (poly[j].x - poly[i].x) * (q.y - poly[i].y) / (poly[j].y - poly[i].y) + poly[i].x) in = !in;
    return in;
}

// Next code point of UTF-8 text (bad bytes read as themselves).
std::uint32_t nextCodePoint(const std::string& s, std::size_t& i) {
    const auto c = static_cast<unsigned char>(s[i++]);
    int more = c >= 0xF0 ? 3 : c >= 0xE0 ? 2 : c >= 0xC0 ? 1 : 0;
    std::uint32_t v = more == 3 ? c & 7u : more == 2 ? c & 15u : more == 1 ? c & 31u : c;
    for (; more > 0 && i < s.size() && (static_cast<unsigned char>(s[i]) & 0xC0) == 0x80; --more) v = (v << 6) | (static_cast<unsigned char>(s[i++]) & 63u);
    return v;
}

} // namespace

bool textFontNamed(const std::string& name, TextFont& font) {
    std::string n;
    for (const char c : name) n += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (n == "sans" || n == "regular" || n == "inter") font = TextFont::Sans;
    else if (n == "bold" || n == "semibold" || n == "sansbold") font = TextFont::SansBold;
    else if (n == "mono" || n == "monospace") font = TextFont::Mono;
    else return false;
    return true;
}

std::vector<std::vector<Vec2>> textOutline(const std::string& text, float height, TextFont font, float* width, std::string* why) {
    std::vector<std::vector<Vec2>> out;
    if (width) *width = 0;
    const TrueType tt(font == TextFont::Mono ? fonts::JetBrainsMono_Regular : font == TextFont::Sans ? fonts::Inter_Regular : fonts::Inter_SemiBold);
    if (!tt.ok()) {
        if (why) *why = "the built-in font could not be read";
        return out;
    }
    if (!(height > 0)) {
        if (why) *why = "the text needs a height";
        return out;
    }
    // Capitals `height` tall: the top of 'H'.
    float cap = 0.7f * tt.unitsPerEm();
    {
        std::vector<std::vector<Vec2>> h;
        tt.contours(tt.glyphOf('H'), h);
        float top = 0;
        for (const auto& c : h)
            for (const Vec2& p : c) top = std::max(top, p.y);
        if (top > 0) cap = top;
    }
    const float scale = height / cap;
    float x = 0, y = 0;
    std::string missing;
    for (std::size_t i = 0; i < text.size();) {
        const std::uint32_t c = nextCodePoint(text, i);
        if (c == '\n') {
            if (width) *width = std::max(*width, x * scale);
            x = 0, y -= 1.6f * cap;
            continue;
        }
        const int g = tt.glyphOf(c);
        if (!g && c != ' ') {
            if (missing.empty()) missing = c < 128 ? std::string(1, static_cast<char>(c)) : "U+" + std::to_string(c);
            continue;
        }
        std::vector<std::vector<Vec2>> glyph;
        const float m[4] = {scale, 0, 0, scale};
        tt.contours(g, glyph, m, x * scale, y * scale);
        // Outer contours counter-clockwise, holes (inside an odd number of others) clockwise.
        std::vector<int> depth(glyph.size(), 0);
        for (std::size_t k = 0; k < glyph.size(); ++k)
            for (std::size_t j = 0; j < glyph.size(); ++j)
                if (j != k && std::fabs(areaOf(glyph[j])) > std::fabs(areaOf(glyph[k])) && inside(glyph[j], glyph[k][0])) ++depth[k];
        for (std::size_t k = 0; k < glyph.size(); ++k) {
            if ((areaOf(glyph[k]) > 0) == (depth[k] % 2 == 1)) std::reverse(glyph[k].begin(), glyph[k].end());
            out.push_back(std::move(glyph[k]));
        }
        x += tt.advance(g);
    }
    if (width) *width = std::max(*width, x * scale);
    if (out.empty() && why) *why = missing.empty() ? "the text has nothing to draw" : "the font has no " + missing;
    return out;
}

} // namespace sicdm
