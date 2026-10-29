#pragma once

// A small XML reader for the exchange formats that are XML (3MF, AMF): elements, attributes
// and text, entities decoded, comments, processing instructions, CDATA and the doctype
// skipped. Names keep only their local part ("m:color" is "color").

#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace sicdm {

struct XmlNode {
    std::string name; // local name
    std::vector<std::pair<std::string, std::string>> attrs; // local names, decoded values
    std::string text; // the text directly in it, decoded (not its children's)
    std::vector<XmlNode> children;

    const std::string* attr(std::string_view local) const;
    std::string attrOr(std::string_view local, std::string fallback = {}) const {
        const std::string* a = attr(local);
        return a ? *a : fallback;
    }
    const XmlNode* child(std::string_view local) const;
    std::vector<const XmlNode*> childrenNamed(std::string_view local) const;
};

// The document's root element. Nothing (with why) when it isn't well-formed.
std::optional<XmlNode> parseXml(std::string_view text, std::string* why = nullptr);

} // namespace sicdm
