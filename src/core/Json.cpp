#include "core/Json.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace sicdm {

namespace {

const std::string kEmptyString;
const Json::Array kEmptyArray;
const Json::Object kEmptyObject;
const Json kNull;

void dumpString(std::string& out, const std::string& s) {
    out += '"';
    for (const char ch : s) {
        const auto c = static_cast<unsigned char>(ch);
        switch (c) {
        case '"': out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        case '\b': out += "\\b"; break;
        case '\f': out += "\\f"; break;
        default:
            if (c < 0x20) {
                char buf[8];
                std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                out += buf;
            } else {
                out += ch;
            }
        }
    }
    out += '"';
}

class Parser {
public:
    explicit Parser(std::string_view s) : m_s(s) {}

    std::optional<Json> parseDocument(std::string* error) {
        skipWs();
        auto v = parseValue(0);
        skipWs();
        if (v && m_pos != m_s.size()) fail("trailing characters");
        if (!m_error.empty()) {
            if (error) *error = m_error + " at offset " + std::to_string(m_pos);
            return std::nullopt;
        }
        return v;
    }

private:
    static constexpr int kMaxDepth = 256;

    void fail(const char* msg) {
        if (m_error.empty()) m_error = msg;
    }
    bool eof() const { return m_pos >= m_s.size(); }
    char peek() const { return eof() ? '\0' : m_s[m_pos]; }
    void skipWs() {
        while (!eof() && (peek() == ' ' || peek() == '\t' || peek() == '\n' || peek() == '\r')) ++m_pos;
    }
    bool consume(std::string_view word) {
        if (m_s.substr(m_pos, word.size()) != word) return false;
        m_pos += word.size();
        return true;
    }

    std::optional<Json> parseValue(int depth) {
        if (depth > kMaxDepth) {
            fail("nesting too deep");
            return std::nullopt;
        }
        switch (peek()) {
        case '{': return parseObject(depth);
        case '[': return parseArray(depth);
        case '"': {
            auto s = parseString();
            if (!s) return std::nullopt;
            return Json(std::move(*s));
        }
        case 't': if (consume("true")) return Json(true); break;
        case 'f': if (consume("false")) return Json(false); break;
        case 'n': if (consume("null")) return Json(nullptr); break;
        default:
            if (peek() == '-' || (peek() >= '0' && peek() <= '9')) return parseNumber();
        }
        fail("unexpected character");
        return std::nullopt;
    }

    std::optional<Json> parseNumber() {
        const std::size_t start = m_pos;
        if (peek() == '-') ++m_pos;
        while (!eof() && ((peek() >= '0' && peek() <= '9') || peek() == '.' || peek() == 'e' || peek() == 'E' ||
                          peek() == '+' || peek() == '-'))
            ++m_pos;
        const std::string text(m_s.substr(start, m_pos - start));
        char* end = nullptr;
        const double v = std::strtod(text.c_str(), &end);
        if (end != text.c_str() + text.size() || !std::isfinite(v)) {
            fail("invalid number");
            return std::nullopt;
        }
        return Json(v);
    }

    static void appendUtf8(std::string& out, unsigned cp) {
        if (cp < 0x80) out += static_cast<char>(cp);
        else if (cp < 0x800) {
            out += static_cast<char>(0xC0 | (cp >> 6));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        } else if (cp < 0x10000) {
            out += static_cast<char>(0xE0 | (cp >> 12));
            out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        } else {
            out += static_cast<char>(0xF0 | (cp >> 18));
            out += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
            out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        }
    }

    std::optional<unsigned> parseHex4() {
        if (m_pos + 4 > m_s.size()) return std::nullopt;
        unsigned v = 0;
        for (int i = 0; i < 4; ++i) {
            const char c = m_s[m_pos++];
            v <<= 4;
            if (c >= '0' && c <= '9') v |= static_cast<unsigned>(c - '0');
            else if (c >= 'a' && c <= 'f') v |= static_cast<unsigned>(c - 'a' + 10);
            else if (c >= 'A' && c <= 'F') v |= static_cast<unsigned>(c - 'A' + 10);
            else return std::nullopt;
        }
        return v;
    }

    std::optional<std::string> parseString() {
        ++m_pos; // opening quote
        std::string out;
        while (!eof()) {
            const char c = m_s[m_pos++];
            if (c == '"') return out;
            if (static_cast<unsigned char>(c) < 0x20) {
                fail("control character in string");
                return std::nullopt;
            }
            if (c != '\\') {
                out += c;
                continue;
            }
            if (eof()) break;
            const char e = m_s[m_pos++];
            switch (e) {
            case '"': out += '"'; break;
            case '\\': out += '\\'; break;
            case '/': out += '/'; break;
            case 'b': out += '\b'; break;
            case 'f': out += '\f'; break;
            case 'n': out += '\n'; break;
            case 'r': out += '\r'; break;
            case 't': out += '\t'; break;
            case 'u': {
                auto cp = parseHex4();
                if (!cp) {
                    fail("invalid \\u escape");
                    return std::nullopt;
                }
                if (*cp >= 0xD800 && *cp <= 0xDBFF && consume("\\u")) { // surrogate pair
                    auto lo = parseHex4();
                    if (!lo || *lo < 0xDC00 || *lo > 0xDFFF) {
                        fail("invalid surrogate pair");
                        return std::nullopt;
                    }
                    *cp = 0x10000 + ((*cp - 0xD800) << 10) + (*lo - 0xDC00);
                }
                appendUtf8(out, *cp);
                break;
            }
            default: fail("invalid escape"); return std::nullopt;
            }
        }
        fail("unterminated string");
        return std::nullopt;
    }

    std::optional<Json> parseArray(int depth) {
        ++m_pos;
        Json::Array a;
        skipWs();
        if (peek() == ']') {
            ++m_pos;
            return Json(std::move(a));
        }
        while (true) {
            skipWs();
            auto v = parseValue(depth + 1);
            if (!v) return std::nullopt;
            a.push_back(std::move(*v));
            skipWs();
            if (peek() == ',') { ++m_pos; continue; }
            if (peek() == ']') { ++m_pos; return Json(std::move(a)); }
            fail("expected ',' or ']'");
            return std::nullopt;
        }
    }

    std::optional<Json> parseObject(int depth) {
        ++m_pos;
        Json::Object o;
        skipWs();
        if (peek() == '}') {
            ++m_pos;
            return Json(std::move(o));
        }
        while (true) {
            skipWs();
            if (peek() != '"') {
                fail("expected string key");
                return std::nullopt;
            }
            auto key = parseString();
            if (!key) return std::nullopt;
            skipWs();
            if (peek() != ':') {
                fail("expected ':'");
                return std::nullopt;
            }
            ++m_pos;
            skipWs();
            auto v = parseValue(depth + 1);
            if (!v) return std::nullopt;
            o[std::move(*key)] = std::move(*v);
            skipWs();
            if (peek() == ',') { ++m_pos; continue; }
            if (peek() == '}') { ++m_pos; return Json(std::move(o)); }
            fail("expected ',' or '}'");
            return std::nullopt;
        }
    }

    std::string_view m_s;
    std::size_t m_pos = 0;
    std::string m_error;
};

} // namespace

const std::string& Json::asString() const noexcept { return isString() ? m_string : kEmptyString; }
const Json::Array& Json::items() const noexcept { return isArray() ? *m_array : kEmptyArray; }
const Json::Object& Json::members() const noexcept { return isObject() ? *m_object : kEmptyObject; }
std::size_t Json::size() const noexcept { return isArray() ? m_array->size() : (isObject() ? m_object->size() : 0); }

const Json& Json::operator[](std::size_t i) const noexcept {
    return isArray() && i < m_array->size() ? (*m_array)[i] : kNull;
}

bool Json::has(const std::string& key) const noexcept { return isObject() && m_object->count(key) > 0; }

const Json& Json::operator[](const std::string& key) const noexcept {
    if (!isObject()) return kNull;
    const auto it = m_object->find(key);
    return it == m_object->end() ? kNull : it->second;
}

Json::Array& Json::mutableArray() {
    if (!isArray()) *this = Json(Array{});
    else if (m_array.use_count() > 1) m_array = std::make_shared<Array>(*m_array); // copy on write
    return *m_array;
}

Json::Object& Json::mutableObject() {
    if (!isObject()) *this = Json(Object{});
    else if (m_object.use_count() > 1) m_object = std::make_shared<Object>(*m_object);
    return *m_object;
}

Json& Json::push(Json v) {
    mutableArray().push_back(std::move(v));
    return *this;
}

Json& Json::set(const std::string& key, Json v) {
    mutableObject()[key] = std::move(v);
    return *this;
}

void Json::dumpTo(std::string& out) const {
    switch (m_type) {
    case Type::Null: out += "null"; break;
    case Type::Bool: out += m_bool ? "true" : "false"; break;
    case Type::Number: {
        char buf[32];
        if (m_number == std::floor(m_number) && std::fabs(m_number) < 1e15)
            std::snprintf(buf, sizeof(buf), "%.0f", m_number);
        else
            std::snprintf(buf, sizeof(buf), "%.9g", m_number);
        out += buf;
        break;
    }
    case Type::String: dumpString(out, m_string); break;
    case Type::Array: {
        out += '[';
        bool first = true;
        for (const Json& v : *m_array) {
            if (!first) out += ',';
            first = false;
            v.dumpTo(out);
        }
        out += ']';
        break;
    }
    case Type::Object: {
        out += '{';
        bool first = true;
        for (const auto& [k, v] : *m_object) {
            if (!first) out += ',';
            first = false;
            dumpString(out, k);
            out += ':';
            v.dumpTo(out);
        }
        out += '}';
        break;
    }
    }
}

std::string Json::dump() const {
    std::string out;
    dumpTo(out);
    return out;
}

std::optional<Json> Json::parse(std::string_view text, std::string* error) {
    return Parser(text).parseDocument(error);
}

} // namespace sicdm
