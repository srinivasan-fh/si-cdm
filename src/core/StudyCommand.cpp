// STUDY: a motion study over time, with a summary, a CSV table and an SVG chart.

#include "core/StudyCommand.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Study.hpp"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>

namespace sicdm {

namespace {

std::string lowerOf(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

bool writeFile(const std::string& path, const std::string& text) {
    std::ofstream f(path, std::ios::binary);
    if (!f) return false;
    f << text;
    return static_cast<bool>(f);
}

} // namespace

CommandOutput runStudyCommand(Document& doc, const std::vector<std::string>& a, const std::vector<bool>& quoted) {
    CommandOutput out;
    const char* usage = "STUDY [motion...] [POINT body [x,y,z]]... [FROM t0] [TO t1] [STEP dt | SAMPLES n] [SMOOTH s] [CSV file] [SVG file] [TITLE text]";
    StudyRequest rq;
    std::string csv, svg, title;
    std::size_t samples = 0;
    auto unq = [&](std::size_t i) { return !(i < quoted.size() && quoted[i]); };
    for (std::size_t i = 0; i < a.size(); ++i) {
        const std::string k = lowerOf(a[i]);
        const bool key = unq(i);
        auto next = [&]() -> const std::string* { return i + 1 < a.size() ? &a[++i] : nullptr; };
        if (key && (k == "from" || k == "to" || k == "step" || k == "smooth")) {
            const std::string* v = next();
            const auto x = v ? parseNumber(*v) : std::nullopt;
            if (!x || *x < 0) return out.fail(toUpper(k) + " needs seconds (>= 0)");
            (k == "from" ? rq.from : k == "to" ? rq.to : k == "step" ? rq.step : rq.smooth) = *x;
        } else if (key && k == "samples") {
            const std::string* v = next();
            const auto x = v ? parseNumber(*v) : std::nullopt;
            if (!x || *x < 3 || *x > 20001) return out.fail("SAMPLES is from 3 to 20001");
            samples = static_cast<std::size_t>(std::lround(*x));
        } else if (key && k == "point") {
            const std::string* body = next();
            if (!body) return out.fail("POINT needs a body (and x,y,z on it, in the model)");
            StudyRequest::Point p;
            p.body = *body;
            if (i + 1 < a.size() && unq(i + 1)) {
                if (const auto at = parseVec3(a[i + 1])) p.at = *at, p.given = true, ++i;
            }
            rq.points.push_back(p);
        } else if (key && (k == "csv" || k == "svg" || k == "title")) {
            const std::string* v = next();
            if (!v) return out.fail(toUpper(k) + (k == "title" ? " needs the chart's title" : " needs a file name"));
            (k == "csv" ? csv : k == "svg" ? svg : title) = *v;
        } else {
            rq.motions.push_back(a[i]); // a motion's name
        }
    }
    if (samples > 0) {
        // The span is only known once the study knows its length: sample it evenly.
        StudyRequest probe = rq;
        StudyResult first;
        std::string why;
        if (!runStudy(doc.scene, probe, first, &why)) return out.fail(why + ".  " + usage);
        rq.from = first.t.front(), rq.to = first.t.back(), rq.step = (rq.to - rq.from) / static_cast<double>(samples - 1);
    }
    auto result = std::make_shared<StudyResult>();
    std::string why;
    if (!runStudy(doc.scene, rq, *result, &why)) return out.fail(why + ".  " + usage);
    char span[96];
    std::snprintf(span, sizeof span, "Motion study, %zu samples from %g to %g s:", result->t.size(), result->t.front(), result->t.back());
    out.say(span);
    for (const std::string& n : result->notes) out.say("  " + n);
    if (!csv.empty()) {
        if (!writeFile(csv, studyCsv(*result))) return out.fail("Can't write " + csv);
        out.say("Wrote " + csv + " (" + std::to_string(result->series.size()) + " columns after the time)");
    }
    if (!svg.empty()) {
        if (!writeFile(svg, studySvg(*result, title.empty() ? std::string("Motion study") : title))) return out.fail("Can't write " + svg);
        out.say("Wrote the chart " + svg);
    }
    doc.study = std::move(result);
    std::string line = "STUDY";
    for (std::size_t i = 0; i < a.size(); ++i) line += " " + (unq(i) ? a[i] : "\"" + a[i] + "\"");
    doc.studyCommand = line;
    return out;
}

} // namespace sicdm
