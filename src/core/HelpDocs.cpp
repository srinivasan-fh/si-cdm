#include "core/HelpDocs.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>

namespace sicdm {

namespace {

std::string joined(const EmbeddedFile& f) {
    std::string s;
    for (const char* p : f.pieces) s += p;
    return s;
}

std::string trimmed(std::string s) {
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.pop_back();
    std::size_t i = 0;
    while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
    return s.substr(i);
}

// The links in a line: [text](target), images excluded.
void collectLinks(const std::string& s, std::vector<std::pair<std::string, std::string>>& out) {
    for (std::size_t i = 0; i < s.size(); ++i) {
        if (s[i] != '[' || (i > 0 && s[i - 1] == '!')) continue;
        const std::size_t close = s.find(']', i);
        if (close == std::string::npos || close + 1 >= s.size() || s[close + 1] != '(') continue;
        const std::size_t end = s.find(')', close);
        if (end == std::string::npos) continue;
        out.push_back({markdownPlain(s.substr(i + 1, close - i - 1)), s.substr(close + 2, end - close - 2)});
        i = end;
    }
}

std::vector<std::string> cells(const std::string& line) {
    std::vector<std::string> out;
    std::string t = trimmed(line);
    if (!t.empty() && t.front() == '|') t.erase(0, 1);
    if (!t.empty() && t.back() == '|') t.pop_back();
    std::string cur;
    for (std::size_t i = 0; i < t.size(); ++i) {
        if (t[i] == '\\' && i + 1 < t.size() && t[i + 1] == '|') {
            cur += '|';
            ++i;
        } else if (t[i] == '|') {
            out.push_back(trimmed(cur));
            cur.clear();
        } else cur += t[i];
    }
    out.push_back(trimmed(cur));
    return out;
}

} // namespace

std::string markdownPlain(const std::string& s) {
    // Emphasis marks are dropped only in pairs: an opening run of one or two
    // stars (or underscores at a word start) before a non-space, closed by the
    // same run after a non-space. A lone star ("x * y", "2*3") stays.
    std::vector<bool> drop(s.size(), false);
    bool code = false;
    for (std::size_t i = 0; i < s.size(); ++i) {
        const char c = s[i];
        if (c == '`') { code = !code; drop[i] = true; continue; }
        if (code || drop[i] || (c != '*' && c != '_')) continue;
        if (c == '_' && i > 0 && std::isalnum(static_cast<unsigned char>(s[i - 1]))) continue;
        const std::size_t n = (i + 1 < s.size() && s[i + 1] == c) ? 2 : 1;
        if (i + n >= s.size() || std::isspace(static_cast<unsigned char>(s[i + n]))) continue;
        for (std::size_t j = i + n; j + n <= s.size(); ++j) {
            if (s[j] == '`') break;
            if (s.compare(j, n, std::string(n, c)) != 0) continue;
            if (std::isspace(static_cast<unsigned char>(s[j - 1]))) continue;
            if (j + n < s.size() && s[j + n] == c) { ++j; continue; }
            if (c == '_' && j + n < s.size() && std::isalnum(static_cast<unsigned char>(s[j + n]))) continue;
            for (std::size_t k = 0; k < n; ++k) drop[i + k] = drop[j + k] = true;
            break;
        }
        i += n - 1;
    }
    std::string out;
    for (std::size_t i = 0; i < s.size(); ++i) {
        const char c = s[i];
        if (drop[i]) continue;
        if (c == '!' && i + 1 < s.size() && s[i + 1] == '[') continue;
        if (c == '[') {
            const std::size_t close = s.find(']', i);
            if (close != std::string::npos && close + 1 < s.size() && s[close + 1] == '(') {
                const std::size_t end = s.find(')', close);
                if (end != std::string::npos) {
                    out += markdownPlain(s.substr(i + 1, close - i - 1));
                    i = end;
                    continue;
                }
            }
        }
        out += c;
    }
    return out;
}

std::vector<MdBlock> parseMarkdown(const std::string& text) {
    std::vector<MdBlock> out;
    std::vector<std::string> lines;
    {
        std::istringstream in(text);
        std::string l;
        while (std::getline(in, l)) {
            if (!l.empty() && l.back() == '\r') l.pop_back();
            lines.push_back(l);
        }
    }
    auto indentOf = [](const std::string& l) {
        std::size_t i = 0;
        while (i < l.size() && l[i] == ' ') ++i;
        return i;
    };
    std::size_t i = 0;
    while (i < lines.size()) {
        const std::string& raw = lines[i];
        const std::string t = trimmed(raw);
        if (t.empty()) {
            ++i;
            continue;
        }
        MdBlock b;
        if (t.rfind("```", 0) == 0) { // fenced code
            b.kind = MdBlock::Code;
            ++i;
            while (i < lines.size() && trimmed(lines[i]).rfind("```", 0) != 0) b.text += lines[i++] + "\n";
            ++i;
            if (!b.text.empty()) b.text.pop_back();
            out.push_back(std::move(b));
            continue;
        }
        if (t[0] == '#') {
            std::size_t n = 0;
            while (n < t.size() && t[n] == '#') ++n;
            b.kind = MdBlock::Heading;
            b.level = static_cast<int>(std::min<std::size_t>(n, 6));
            b.text = trimmed(t.substr(n));
            collectLinks(b.text, b.links);
            out.push_back(std::move(b));
            ++i;
            continue;
        }
        if (t == "---" || t == "***") {
            b.kind = MdBlock::Rule;
            out.push_back(std::move(b));
            ++i;
            continue;
        }
        if (t.rfind("![", 0) == 0) {
            const std::size_t close = t.find("](");
            const std::size_t end = t.rfind(')');
            b.kind = MdBlock::Image;
            if (close != std::string::npos && end != std::string::npos && end > close) {
                b.text = t.substr(2, close - 2);
                b.target = t.substr(close + 2, end - close - 2);
            }
            out.push_back(std::move(b));
            ++i;
            continue;
        }
        if (t[0] == '|') {
            b.kind = MdBlock::Table;
            while (i < lines.size() && !trimmed(lines[i]).empty() && trimmed(lines[i])[0] == '|') {
                const std::string row = trimmed(lines[i++]);
                if (row.find_first_not_of("|-: ") == std::string::npos) continue; // the header's rule
                b.rows.push_back(cells(row));
                collectLinks(row, b.links);
            }
            out.push_back(std::move(b));
            continue;
        }
        const bool bullet = (t[0] == '-' || t[0] == '*' || t[0] == '+') && t.size() > 1 && t[1] == ' ';
        std::size_t digits = 0;
        while (digits < t.size() && std::isdigit(static_cast<unsigned char>(t[digits]))) ++digits;
        const bool numbered = digits > 0 && digits + 1 < t.size() && t[digits] == '.' && t[digits + 1] == ' ';
        if (bullet || numbered) {
            b.kind = bullet ? MdBlock::Bullet : MdBlock::Numbered;
            const std::size_t ind = indentOf(raw);
            b.level = static_cast<int>(ind / 2);
            b.text = bullet ? trimmed(t.substr(2)) : t; // a numbered item keeps its number
            ++i;
            // Its continuation lines: indented further, not a new item or block.
            while (i < lines.size()) {
                const std::string n = trimmed(lines[i]);
                if (n.empty() || indentOf(lines[i]) <= ind) break;
                const bool item = ((n[0] == '-' || n[0] == '*' || n[0] == '+') && n.size() > 1 && n[1] == ' ') || n.rfind("```", 0) == 0 || n[0] == '|';
                std::size_t d = 0;
                while (d < n.size() && std::isdigit(static_cast<unsigned char>(n[d]))) ++d;
                if (item || (d > 0 && d + 1 < n.size() && n[d] == '.' && n[d + 1] == ' ')) break;
                b.text += " " + n;
                ++i;
            }
            collectLinks(b.text, b.links);
            out.push_back(std::move(b));
            continue;
        }
        if (t[0] == '>') {
            b.kind = MdBlock::Quote;
            while (i < lines.size() && !trimmed(lines[i]).empty() && trimmed(lines[i])[0] == '>')
                b.text += (b.text.empty() ? "" : " ") + trimmed(trimmed(lines[i++]).substr(1));
            collectLinks(b.text, b.links);
            out.push_back(std::move(b));
            continue;
        }
        // A paragraph: up to a blank line or another block.
        b.kind = MdBlock::Paragraph;
        while (i < lines.size()) {
            const std::string n = trimmed(lines[i]);
            if (n.empty() || n[0] == '#' || n.rfind("```", 0) == 0 || n[0] == '|' || n.rfind("![", 0) == 0 || n[0] == '>' ||
                ((n[0] == '-' || n[0] == '*') && n.size() > 1 && n[1] == ' '))
                break;
            b.text += (b.text.empty() ? "" : " ") + n;
            ++i;
        }
        collectLinks(b.text, b.links);
        out.push_back(std::move(b));
    }
    return out;
}

const std::vector<HelpDoc>& helpDocs() {
    static const std::vector<HelpDoc> docs = [] {
        std::vector<HelpDoc> v;
        for (const EmbeddedFile& f : embeddedDocFiles()) {
            const std::string path = f.path;
            if (path.size() < 3 || path.compare(path.size() - 3, 3, ".md") != 0 || path.find('/') != std::string::npos) continue;
            HelpDoc d;
            d.file = path;
            d.text = joined(f);
            for (const MdBlock& b : parseMarkdown(d.text))
                if (b.kind == MdBlock::Heading) {
                    d.title = markdownPlain(b.text);
                    break;
                }
            if (d.title.empty()) d.title = path;
            v.push_back(std::move(d));
        }
        std::stable_sort(v.begin(), v.end(), [](const HelpDoc& a, const HelpDoc& b) {
            const bool sa = a.file == "getting-started.md", sb = b.file == "getting-started.md";
            if (sa != sb) return sa;
            return a.title < b.title;
        });
        return v;
    }();
    return docs;
}

const HelpDoc* helpDoc(const std::string& file) {
    for (const HelpDoc& d : helpDocs())
        if (d.file == file) return &d;
    return nullptr;
}

const std::vector<HelpExample>& helpExamples() {
    static const std::vector<HelpExample> ex = [] {
        std::vector<HelpExample> v;
        for (const EmbeddedFile& f : embeddedDocFiles()) {
            const std::string path = f.path;
            if (path.rfind("examples/", 0) != 0 || path.size() < 4 || path.compare(path.size() - 4, 4, ".txt") != 0) continue;
            HelpExample e;
            e.file = path;
            e.text = joined(f);
            const std::size_t nl = e.text.find('\n');
            const std::string first = trimmed(e.text.substr(0, nl));
            e.title = first.rfind('#', 0) == 0 ? trimmed(first.substr(1)) : path.substr(9);
            v.push_back(std::move(e));
        }
        return v;
    }();
    return ex;
}

} // namespace sicdm
