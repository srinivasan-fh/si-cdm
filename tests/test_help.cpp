#include "TestFramework.hpp"

#include "core/HelpDocs.hpp"

#include <cctype>
#include <cstdio>
#include <filesystem>
#include <set>
#include <string>

using namespace sicdm;

#define REQUIRE(c)                  \
    do {                            \
        const bool required_ = (c); \
        CHECK(required_);           \
        if (!required_) return;     \
    } while (0)

namespace {

std::string slugOf(const std::string& heading) {
    std::string out;
    for (char c : markdownPlain(heading)) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        if (std::isalnum(static_cast<unsigned char>(c)) || c == '-' || c == '_') out += c;
        else if (c == ' ') out += '-';
    }
    return out;
}

} // namespace

TEST_CASE("Help: the user guide is built in, Getting started first, and every page parses") {
    const auto& docs = helpDocs();
    REQUIRE(docs.size() >= 10);
    CHECK(docs.front().file == "getting-started.md");
    CHECK(docs.front().title == "Getting started");
    for (const char* f : {"cam.md", "lathe.md", "cad.md", "drawing.md", "step.md", "licensing.md", "plugins.md"}) CHECK(helpDoc(f) != nullptr);
    for (const HelpDoc& d : docs) {
        const auto blocks = parseMarkdown(d.text);
        CHECK(!blocks.empty());
        CHECK(blocks.front().kind == MdBlock::Heading);
        CHECK(!d.title.empty());
        // Every code fence closed (the parser would swallow the rest of the page).
        std::size_t fences = 0;
        for (std::size_t p = d.text.find("```"); p != std::string::npos; p = d.text.find("```", p + 3)) ++fences;
        CHECK(fences % 2 == 0);
    }
    const auto& ex = helpExamples();
    CHECK(ex.size() >= 4);
    for (const HelpExample& e : ex) CHECK(!e.title.empty());
}

TEST_CASE("Help: every link between pages leads to a page and heading that exist") {
    std::size_t checked = 0;
    for (const HelpDoc& d : helpDocs()) {
        std::set<std::string> anchors;
        for (const MdBlock& b : parseMarkdown(d.text))
            if (b.kind == MdBlock::Heading) anchors.insert(slugOf(b.text));
        for (const MdBlock& b : parseMarkdown(d.text))
            for (const auto& [text, target] : b.links) {
                if (target.rfind("http", 0) == 0 || target.rfind("mailto:", 0) == 0) continue;
                const std::size_t hash = target.find('#');
                const std::string file = target.substr(0, hash), anchor = hash == std::string::npos ? "" : target.substr(hash + 1);
                ++checked;
                if (file.size() > 3 && file.compare(file.size() - 3, 3, ".md") == 0) {
                    const HelpDoc* to = helpDoc(file);
                    if (!to) std::printf("  %s: link to %s\n", d.file.c_str(), target.c_str());
                    CHECK(to != nullptr);
                    if (to && !anchor.empty()) {
                        bool found = false;
                        for (const MdBlock& h : parseMarkdown(to->text))
                            if (h.kind == MdBlock::Heading && slugOf(h.text) == anchor) found = true;
                        if (!found) std::printf("  %s: no heading #%s in %s\n", d.file.c_str(), anchor.c_str(), file.c_str());
                        CHECK(found);
                    }
                } else if (file.empty()) {
                    if (!anchors.count(anchor)) std::printf("  %s: no heading #%s\n", d.file.c_str(), anchor.c_str());
                    CHECK(anchors.count(anchor) == 1);
                } else {
                    // Files beside the guide (examples, pictures).
                    const bool there = std::filesystem::exists(std::filesystem::path(SICDM_SOURCE_DIR) / "docs" / file);
                    if (!there) std::printf("  %s: missing %s\n", d.file.c_str(), target.c_str());
                    CHECK(there);
                }
            }
    }
    CHECK(checked > 20);
}

TEST_CASE("Help: Markdown blocks and inline marks") {
    const auto b = parseMarkdown("# Title\n\nSome **bold** and `code`, see [CAM](cam.md#setup).\n\n- one\n  more\n  - two\n1. first\n\n```text\nCAM LIST\n```\n\n| a | b |\n| --- | --- |\n| 1 | x \\| y |\n\n![A picture](pic.png)\n\n> Careful\n");
    REQUIRE(b.size() == 9);
    CHECK(b[0].kind == MdBlock::Heading);
    CHECK(b[1].kind == MdBlock::Paragraph);
    CHECK(markdownPlain(b[1].text) == "Some bold and code, see CAM.");
    REQUIRE(b[1].links.size() == 1);
    CHECK(b[1].links[0].second == "cam.md#setup");
    CHECK(b[2].kind == MdBlock::Bullet);
    CHECK(b[2].text == "one more");
    CHECK(b[3].level == 1);
    CHECK(b[4].kind == MdBlock::Numbered);
    CHECK(b[5].kind == MdBlock::Code);
    CHECK(b[5].text == "CAM LIST");
    CHECK(b[6].kind == MdBlock::Table);
    REQUIRE(b[6].rows.size() == 2);
    CHECK(b[6].rows[1][1] == "x | y");
    CHECK(b[7].kind == MdBlock::Image);
    CHECK(b[7].target == "pic.png");
    CHECK(b[8].kind == MdBlock::Quote);
    CHECK(markdownPlain("x * y and 2*3") == "x * y and 2*3");
}
