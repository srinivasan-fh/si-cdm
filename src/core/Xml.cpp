// A small XML reader (Xml.hpp).

#include "core/Xml.hpp"

#include <cstdlib>
#include <cstring>

namespace sicdm {

namespace {

bool nameChar(char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' || c == ':' || c == '-' || c == '.' ||
           static_cast<unsigned char>(c) >= 0x80;
}
bool space(char c) { return c == ' ' || c == '\t' || c == '\n' || c == '\r'; }

std::string localName(std::string_view q) {
    const std::size_t colon = q.rfind(':');
    return std::string(colon == std::string_view::npos ? q : q.substr(colon + 1));
}

void appendUtf8(std::string& out, unsigned long cp) {
    if (cp < 0x80) out.push_back(static_cast<char>(cp));
    else if (cp < 0x800) {
        out.push_back(static_cast<char>(0xC0 | (cp >> 6)));
        out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
    } else if (cp < 0x10000) {
        out.push_back(static_cast<char>(0xE0 | (cp >> 12)));
        out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
        out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
    } else {
        out.push_back(static_cast<char>(0xF0 | (cp >> 18)));
        out.push_back(static_cast<char>(0x80 | ((cp >> 12) & 0x3F)));
        out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
        out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
    }
}

// Text with its entities decoded.
std::string decode(std::string_view s) {
    if (s.find('&') == std::string_view::npos) return std::string(s);
    std::string out;
    out.reserve(s.size());
    for (std::size_t i = 0; i < s.size(); ++i) {
        if (s[i] != '&') {
            out.push_back(s[i]);
            continue;
        }
        const std::size_t semi = s.find(';', i);
        if (semi == std::string_view::npos || semi - i > 10) {
            out.push_back('&');
            continue;
        }
        const std::string_view e = s.substr(i + 1, semi - i - 1);
        if (e == "lt") out.push_back('<');
        else if (e == "gt") out.push_back('>');
        else if (e == "amp") out.push_back('&');
        else if (e == "quot") out.push_back('"');
        else if (e == "apos") out.push_back('\'');
        else if (!e.empty() && e[0] == '#') {
            const std::string num(e.substr(1));
            const unsigned long cp = num.size() > 1 && (num[0] == 'x' || num[0] == 'X') ? std::strtoul(num.c_str() + 1, nullptr, 16) : std::strtoul(num.c_str(), nullptr, 10);
            appendUtf8(out, cp);
        } else {
            out.append(s.substr(i, semi - i + 1)); // unknown: kept as it is
        }
        i = semi;
    }
    return out;
}

struct Parser {
    std::string_view s;
    std::size_t i = 0;
    std::string why;

    bool failAt(const std::string& w) {
        if (why.empty()) {
            std::size_t line = 1;
            for (std::size_t k = 0; k < i && k < s.size(); ++k) line += s[k] == '\n';
            why = w + " (line " + std::to_string(line) + ")";
        }
        return false;
    }
    void skipSpace() {
        while (i < s.size() && space(s[i])) ++i;
    }
    // Comments, processing instructions and the doctype, and the space between them.
    bool skipMisc() {
        for (;;) {
            skipSpace();
            if (s.compare(i, 4, "<!--") == 0) {
                const std::size_t e = s.find("-->", i + 4);
                if (e == std::string_view::npos) return failAt("a comment isn't closed");
                i = e + 3;
            } else if (s.compare(i, 2, "<?") == 0) {
                const std::size_t e = s.find("?>", i + 2);
                if (e == std::string_view::npos) return failAt("a processing instruction isn't closed");
                i = e + 2;
            } else if (s.compare(i, 9, "<!DOCTYPE") == 0) {
                int depth = 0;
                for (; i < s.size(); ++i) {
                    if (s[i] == '[') ++depth;
                    else if (s[i] == ']') --depth;
                    else if (s[i] == '>' && depth <= 0) break;
                }
                if (i >= s.size()) return failAt("the doctype isn't closed");
                ++i;
            } else {
                return true;
            }
        }
    }
    bool name(std::string_view& out) {
        const std::size_t b = i;
        while (i < s.size() && nameChar(s[i])) ++i;
        if (i == b) return failAt("a name was expected");
        out = s.substr(b, i - b);
        return true;
    }
    bool element(XmlNode& node, int depth) {
        if (depth > 512) return failAt("elements nested too deep");
        if (i >= s.size() || s[i] != '<') return failAt("an element was expected");
        ++i;
        std::string_view q;
        if (!name(q)) return false;
        node.name = localName(q);
        // Attributes.
        for (;;) {
            skipSpace();
            if (i >= s.size()) return failAt("an element isn't closed");
            if (s[i] == '/') {
                if (i + 1 >= s.size() || s[i + 1] != '>') return failAt("'/>' was expected");
                i += 2;
                return true; // empty element
            }
            if (s[i] == '>') {
                ++i;
                break;
            }
            std::string_view an;
            if (!name(an)) return false;
            skipSpace();
            if (i >= s.size() || s[i] != '=') return failAt("'=' was expected after an attribute name");
            ++i;
            skipSpace();
            if (i >= s.size() || (s[i] != '"' && s[i] != '\'')) return failAt("an attribute value must be quoted");
            const char quote = s[i++];
            const std::size_t e = s.find(quote, i);
            if (e == std::string_view::npos) return failAt("an attribute value isn't closed");
            node.attrs.emplace_back(localName(an), decode(s.substr(i, e - i)));
            i = e + 1;
        }
        // Content: text, children, comments, CDATA, up to the end tag.
        for (;;) {
            const std::size_t lt = s.find('<', i);
            if (lt == std::string_view::npos) return failAt("<" + std::string(q) + "> isn't closed");
            if (lt > i) node.text += decode(s.substr(i, lt - i));
            i = lt;
            if (s.compare(i, 2, "</") == 0) {
                i += 2;
                std::string_view end;
                if (!name(end)) return false;
                if (end != q) return failAt("</" + std::string(end) + "> closes <" + std::string(q) + ">");
                skipSpace();
                if (i >= s.size() || s[i] != '>') return failAt("'>' was expected");
                ++i;
                return true;
            }
            if (s.compare(i, 9, "<![CDATA[") == 0) {
                const std::size_t e = s.find("]]>", i + 9);
                if (e == std::string_view::npos) return failAt("a CDATA section isn't closed");
                node.text.append(s.substr(i + 9, e - i - 9));
                i = e + 3;
                continue;
            }
            if (s.compare(i, 4, "<!--") == 0 || s.compare(i, 2, "<?") == 0) {
                if (!skipMisc()) return false;
                continue;
            }
            node.children.emplace_back();
            if (!element(node.children.back(), depth + 1)) return false;
        }
    }
};

} // namespace

const std::string* XmlNode::attr(std::string_view local) const {
    for (const auto& [k, v] : attrs)
        if (k == local) return &v;
    return nullptr;
}

const XmlNode* XmlNode::child(std::string_view local) const {
    for (const XmlNode& c : children)
        if (c.name == local) return &c;
    return nullptr;
}

std::vector<const XmlNode*> XmlNode::childrenNamed(std::string_view local) const {
    std::vector<const XmlNode*> out;
    for (const XmlNode& c : children)
        if (c.name == local) out.push_back(&c);
    return out;
}

std::optional<XmlNode> parseXml(std::string_view text, std::string* why) {
    Parser p;
    p.s = text;
    if (text.size() >= 3 && static_cast<unsigned char>(text[0]) == 0xEF && static_cast<unsigned char>(text[1]) == 0xBB && static_cast<unsigned char>(text[2]) == 0xBF)
        p.i = 3; // a UTF-8 byte order mark
    XmlNode root;
    if (!p.skipMisc() || !p.element(root, 0)) {
        if (why) *why = "not well-formed XML: " + p.why;
        return std::nullopt;
    }
    return root;
}

} // namespace sicdm
