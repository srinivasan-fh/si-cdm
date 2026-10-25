#pragma once

// The user guide inside the app: the docs/*.md files and the example scripts
// (docs/examples/*.txt), built in (cmake/EmbedDocs.cmake), and a reader for the Markdown
// they are written in, enough for the help window to show them.

#include <string>
#include <utility>
#include <vector>

namespace sicdm {

struct EmbeddedFile {
    const char* path;                 // relative to docs/: "cam.md", "examples/cam-demo.txt"
    std::vector<const char*> pieces;  // the text, in pieces
};
const std::vector<EmbeddedFile>& embeddedDocFiles(); // generated

struct HelpDoc {
    std::string file;  // "cam.md"
    std::string title; // its first heading
    std::string text;
};
// The guides, Getting started first, then by title.
const std::vector<HelpDoc>& helpDocs();
const HelpDoc* helpDoc(const std::string& file);

struct HelpExample {
    std::string file;  // "examples/cam-demo.txt"
    std::string title; // its first comment line
    std::string text;  // command lines
};
const std::vector<HelpExample>& helpExamples();

// A Markdown block. Inline marks stay in `text` (see markdownPlain); links are listed.
struct MdBlock {
    enum Kind { Heading, Paragraph, Bullet, Numbered, Code, Table, Image, Rule, Quote } kind = Paragraph;
    int level = 0;    // heading 1-6; list nesting 0, 1, ...
    std::string text; // heading / paragraph / item text, code, image caption
    std::string target; // image file
    std::vector<std::vector<std::string>> rows; // table cells (first row: the header)
    std::vector<std::pair<std::string, std::string>> links; // (text, target) in the text
};
std::vector<MdBlock> parseMarkdown(const std::string& text);
// The text without inline marks: **bold**, *em*, `code`, [text](link) -> text.
std::string markdownPlain(const std::string& text);

} // namespace sicdm
