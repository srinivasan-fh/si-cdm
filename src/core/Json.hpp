#pragma once

// Minimal JSON value with a strict parser and a compact serialiser.
// Used for project files and for MCP (JSON-RPC) messages.

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace sicdm {

class Json {
public:
    enum class Type { Null, Bool, Number, String, Array, Object };
    using Array = std::vector<Json>;
    using Object = std::map<std::string, Json>;

    Json() = default;
    Json(std::nullptr_t) {}
    Json(bool b) : m_type(Type::Bool), m_bool(b) {}
    Json(double n) : m_type(Type::Number), m_number(n) {}
    Json(float n) : Json(static_cast<double>(n)) {}
    Json(int n) : Json(static_cast<double>(n)) {}
    Json(std::int64_t n) : Json(static_cast<double>(n)) {}
    Json(std::size_t n) : Json(static_cast<double>(n)) {}
    Json(const char* s) : m_type(Type::String), m_string(s) {}
    Json(std::string s) : m_type(Type::String), m_string(std::move(s)) {}
    Json(std::string_view s) : m_type(Type::String), m_string(s) {}
    Json(Array a) : m_type(Type::Array), m_array(std::make_shared<Array>(std::move(a))) {}
    Json(Object o) : m_type(Type::Object), m_object(std::make_shared<Object>(std::move(o))) {}

    static Json array() { return Json(Array{}); }
    static Json object() { return Json(Object{}); }

    Type type() const noexcept { return m_type; }
    bool isNull() const noexcept { return m_type == Type::Null; }
    bool isBool() const noexcept { return m_type == Type::Bool; }
    bool isNumber() const noexcept { return m_type == Type::Number; }
    bool isString() const noexcept { return m_type == Type::String; }
    bool isArray() const noexcept { return m_type == Type::Array; }
    bool isObject() const noexcept { return m_type == Type::Object; }

    bool asBool(bool fallback = false) const noexcept { return isBool() ? m_bool : fallback; }
    double asNumber(double fallback = 0.0) const noexcept { return isNumber() ? m_number : fallback; }
    float asFloat(float fallback = 0.0f) const noexcept { return isNumber() ? static_cast<float>(m_number) : fallback; }
    int asInt(int fallback = 0) const noexcept { return isNumber() ? static_cast<int>(m_number) : fallback; }
    const std::string& asString() const noexcept;
    std::string asString(std::string fallback) const { return isString() ? m_string : fallback; }

    // Arrays.
    const Array& items() const noexcept;
    std::size_t size() const noexcept;
    const Json& operator[](std::size_t i) const noexcept;
    const Json& operator[](int i) const noexcept { return i < 0 ? (*this)[std::size_t(-1)] : (*this)[static_cast<std::size_t>(i)]; }
    Json& push(Json v); // converts null to an array

    // Objects.
    const Object& members() const noexcept;
    bool has(const std::string& key) const noexcept;
    const Json& operator[](const std::string& key) const noexcept; // null if missing
    const Json& operator[](const char* key) const noexcept { return (*this)[std::string(key)]; }
    Json& set(const std::string& key, Json v); // converts null to an object

    std::string dump() const;
    static std::optional<Json> parse(std::string_view text, std::string* error = nullptr);

private:
    void dumpTo(std::string& out) const;
    Array& mutableArray();
    Object& mutableObject();

    Type m_type = Type::Null;
    bool m_bool = false;
    double m_number = 0.0;
    std::string m_string;
    std::shared_ptr<Array> m_array;   // shared_ptr keeps Json copyable while recursive
    std::shared_ptr<Object> m_object;
};

} // namespace sicdm
