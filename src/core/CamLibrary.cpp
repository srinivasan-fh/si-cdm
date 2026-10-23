#include "core/CamLibrary.hpp"

#include "core/FileIO.hpp"
#include "core/Json.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace fs = std::filesystem;

namespace sicdm {

namespace {

std::string lowerOf(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

std::string env(const char* name) {
    const char* v = std::getenv(name);
    return v ? std::string(v) : std::string();
}

std::string num(double v) {
    char b[32];
    std::snprintf(b, sizeof b, "%.6g", v);
    return b;
}

bool readText(const std::string& path, std::string& text, std::string* error) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        if (error) *error = "Cannot read " + path;
        return false;
    }
    std::ostringstream s;
    s << f.rdbuf();
    text = s.str();
    return true;
}

// --- CSV ------------------------------------------------------------------------------------

// One CSV record (quoted fields may hold commas and doubled quotes); false at the end.
bool csvRecord(const std::string& text, std::size_t& pos, std::vector<std::string>& fields) {
    fields.clear();
    if (pos >= text.size()) return false;
    std::string cur;
    bool quoted = false, any = false;
    for (; pos < text.size(); ++pos) {
        const char c = text[pos];
        if (quoted) {
            if (c == '"' && pos + 1 < text.size() && text[pos + 1] == '"') cur += '"', ++pos;
            else if (c == '"') quoted = false;
            else cur += c;
            continue;
        }
        if (c == '"') quoted = true, any = true;
        else if (c == ',' || c == ';' || c == '\t') fields.push_back(cur), cur.clear(), any = true;
        else if (c == '\r') continue;
        else if (c == '\n') {
            ++pos;
            break;
        } else cur += c, any = true;
    }
    if (any || !cur.empty()) fields.push_back(cur);
    return true;
}

std::string csvField(const std::string& s) {
    if (s.find_first_of(",\";\n") == std::string::npos) return s;
    std::string o = "\"";
    for (char c : s) o += c == '"' ? std::string("\"\"") : std::string(1, c);
    return o + "\"";
}

std::string trim(const std::string& s) {
    const auto a = s.find_first_not_of(" \t");
    if (a == std::string::npos) return "";
    return s.substr(a, s.find_last_not_of(" \t") - a + 1);
}

// The column a header names, by its name or a common alias.
int columnOf(const std::vector<std::string>& header, std::initializer_list<const char*> names) {
    for (std::size_t i = 0; i < header.size(); ++i) {
        std::string h;
        for (char c : lowerOf(trim(header[i])))
            if (std::isalnum(static_cast<unsigned char>(c))) h += c;
        for (const char* n : names)
            if (h == n) return static_cast<int>(i);
    }
    return -1;
}

bool toolType(const std::string& w, CamToolType* out) {
    const std::string k = lowerOf(trim(w));
    if (camToolTypeFromKey(k, out)) return true;
    static const std::pair<const char*, CamToolType> kAliases[] = {
        {"flat end mill", CamToolType::Flat}, {"end mill", CamToolType::Flat},      {"endmill", CamToolType::Flat},
        {"square", CamToolType::Flat},        {"ball end mill", CamToolType::Ball}, {"ballnose", CamToolType::Ball},
        {"ball nose", CamToolType::Ball},     {"bull nose end mill", CamToolType::BullNose}, {"bullnose", CamToolType::BullNose},
        {"corner radius", CamToolType::BullNose}, {"radius end mill", CamToolType::BullNose}, {"bull nose", CamToolType::BullNose},
        {"bull", CamToolType::BullNose},      {"torus", CamToolType::BullNose},     {"ball", CamToolType::Ball},
        {"flat", CamToolType::Flat},          {"slot drill", CamToolType::Flat},    {"square end mill", CamToolType::Flat},
        {"face mill", CamToolType::FaceMill}, {"shell mill", CamToolType::FaceMill}, {"jobber drill", CamToolType::Drill},
        {"facemill", CamToolType::FaceMill},  {"drill", CamToolType::Drill},        {"twist drill", CamToolType::Drill},
        {"spot drill", CamToolType::Spot},    {"center drill", CamToolType::Spot},  {"centre drill", CamToolType::Spot},
        {"spotdrill", CamToolType::Spot},     {"chamfer mill", CamToolType::Chamfer}, {"chamfer", CamToolType::Chamfer},
        {"countersink", CamToolType::Chamfer}, {"tap", CamToolType::Tap},           {"tap right hand", CamToolType::Tap},
        {"thread tap", CamToolType::Tap}};
    for (const auto& [a, t] : kAliases)
        if (k == a) return *out = t, true;
    return false;
}

// Sensible values for what a catalogue left out, by type (as CAM TOOL fills them in).
void fillDefaults(CamTool& t, bool fluteGiven, bool anglesGiven, bool flutesGiven) {
    if (!flutesGiven)
        t.flutes = t.type == CamToolType::FaceMill ? 5 : t.type == CamToolType::Flat || t.type == CamToolType::BullNose || t.type == CamToolType::Tap ? 3 : 2;
    if (!fluteGiven) t.fluteLength = t.type == CamToolType::FaceMill ? 6.0 : t.type == CamToolType::Drill ? t.diameter * 5 : t.diameter * 3;
    if (!anglesGiven) t.tipAngle = t.type == CamToolType::Drill ? 118.0 : 90.0;
    if (t.type == CamToolType::BullNose && t.cornerRadius <= 0) t.cornerRadius = std::min(1.0, t.diameter * 0.25);
    t.cornerRadius = std::min(t.cornerRadius, t.radius());
}

} // namespace

std::string toolLibraryPath() {
    if (const std::string p = env("SICDM_TOOL_LIBRARY"); !p.empty()) return p;
#if defined(_WIN32)
    if (const std::string d = env("APPDATA"); !d.empty()) return (fs::path(d) / "SI-CDM" / "tools.json").string();
#elif defined(__APPLE__)
    if (const std::string h = env("HOME"); !h.empty()) return (fs::path(h) / "Library" / "Application Support" / "SI-CDM" / "tools.json").string();
#else
    if (const std::string d = env("XDG_CONFIG_HOME"); !d.empty()) return (fs::path(d) / "sicdm" / "tools.json").string();
    if (const std::string h = env("HOME"); !h.empty()) return (fs::path(h) / ".config" / "sicdm" / "tools.json").string();
#endif
    std::error_code ec;
    return (fs::temp_directory_path(ec) / "sicdm-tools.json").string();
}

std::string toolsToCsv(const std::vector<CamTool>& tools) {
    std::string s = "number,type,diameter,flutes,flute_length,corner_radius,tip_angle,pitch,material,name,vendor,product_id,vc,fz,stickout,holder_diameter,"
                    "holder_length\n";
    for (const CamTool& t : tools) {
        s += std::to_string(t.number) + "," + camToolTypeKey(t.type) + "," + num(t.diameter) + "," + std::to_string(t.flutes) + "," + num(t.fluteLength) + "," +
             num(t.cornerRadius) + "," + num(t.tipAngle) + "," + num(t.pitch) + "," + (t.hss ? "hss" : "carbide") + "," + csvField(t.name) + "," +
             csvField(t.vendor) + "," + csvField(t.productId) + "," + (t.vc > 0 ? num(t.vc) : "") + "," + (t.fz > 0 ? num(t.fz) : "") + "," +
             (t.stickout > 0 ? num(t.stickout) : "") + "," + (t.holderDiameter > 0 ? num(t.holderDiameter) : "") + "," + num(t.holderLength) + "\n";
    }
    return s;
}

bool toolsFromCsv(const std::string& text, std::vector<CamTool>& tools, std::string* error, std::vector<std::string>* notes) {
    std::size_t pos = text.size() >= 3 && text.compare(0, 3, "\xEF\xBB\xBF") == 0 ? 3 : 0; // a UTF-8 byte order mark
    std::vector<std::string> header, f;
    if (!csvRecord(text, pos, header) || header.empty()) {
        if (error) *error = "The CSV has no header row";
        return false;
    }
    const int cNumber = columnOf(header, {"number", "tool", "t", "toolnumber", "no", "pocket"});
    const int cType = columnOf(header, {"type", "tooltype", "kind"});
    const int cDia = columnOf(header, {"diameter", "dia", "d", "dc", "cuttingdiameter"});
    const int cFlutes = columnOf(header, {"flutes", "z", "nof", "teeth", "numberofflutes"});
    const int cFlute = columnOf(header, {"flutelength", "lcf", "cuttinglength", "loc", "length"});
    const int cCorner = columnOf(header, {"cornerradius", "corner", "re", "radius"});
    const int cAngle = columnOf(header, {"tipangle", "angle", "pointangle", "sig"});
    const int cPitch = columnOf(header, {"pitch", "tp", "threadpitch"});
    const int cMat = columnOf(header, {"material", "bmc", "toolmaterial", "grade"});
    const int cName = columnOf(header, {"name", "description", "desc"});
    const int cVendor = columnOf(header, {"vendor", "manufacturer", "brand", "maker"});
    const int cProduct = columnOf(header, {"productid", "product", "partnumber", "orderno", "catalog", "catalogue", "sku", "edp"});
    const int cVc = columnOf(header, {"vc", "cuttingspeed", "sfm", "surfacespeed"});
    const int cFz = columnOf(header, {"fz", "chipload", "feedpertooth", "fn"});
    const int cStick = columnOf(header, {"stickout", "gauge", "gaugelength"});
    const int cHolderD = columnOf(header, {"holderdiameter", "holder"});
    const int cHolderL = columnOf(header, {"holderlength"});
    if (cType < 0 || cDia < 0) {
        if (error) *error = "The CSV needs at least a type and a diameter column";
        return false;
    }
    const bool sfm = cVc >= 0 && lowerOf(trim(header[static_cast<std::size_t>(cVc)])) == "sfm"; // feet per minute
    auto at = [&](int c) { return c >= 0 && c < static_cast<int>(f.size()) ? trim(f[static_cast<std::size_t>(c)]) : std::string(); };
    auto number = [&](int c, double& v) {
        const std::string s = at(c);
        if (s.empty()) return false;
        char* end = nullptr;
        const double x = std::strtod(s.c_str(), &end);
        if (end == s.c_str()) return false;
        v = x;
        return true;
    };
    int line = 1, next = 1;
    for (const CamTool& t : tools) next = std::max(next, t.number + 1);
    while (csvRecord(text, pos, f)) {
        ++line;
        if (f.empty() || (f.size() == 1 && trim(f[0]).empty())) continue;
        CamTool t;
        if (!toolType(at(cType), &t.type)) {
            if (notes) notes->push_back("line " + std::to_string(line) + ": unknown tool type '" + at(cType) + "' left out");
            continue;
        }
        double v = 0;
        if (!number(cDia, v) || !(v > 0.05) || v > 300) {
            if (notes) notes->push_back("line " + std::to_string(line) + ": no usable diameter, left out");
            continue;
        }
        t.diameter = v;
        t.number = number(cNumber, v) && v >= 1 && v <= 999 ? static_cast<int>(v) : next;
        next = std::max(next, t.number + 1);
        const bool flutes = number(cFlutes, v) && v >= 1 && v <= 20;
        if (flutes) t.flutes = static_cast<int>(v);
        const bool flute = number(cFlute, v) && v > 0;
        if (flute) t.fluteLength = v;
        if (number(cCorner, v) && v >= 0) t.cornerRadius = v;
        const bool angle = number(cAngle, v) && v >= 20 && v <= 179;
        if (angle) t.tipAngle = v;
        if (number(cPitch, v) && v > 0) t.pitch = v;
        t.hss = lowerOf(at(cMat)).find("hss") != std::string::npos || lowerOf(at(cMat)).find("high speed") != std::string::npos;
        t.name = at(cName), t.vendor = at(cVendor), t.productId = at(cProduct);
        if (number(cVc, v) && v > 0) t.vc = sfm ? v * 0.3048 : v;
        if (number(cFz, v) && v > 0) t.fz = v;
        if (number(cStick, v) && v > 0) t.stickout = v;
        if (number(cHolderD, v) && v > 0) t.holderDiameter = v;
        if (number(cHolderL, v) && v > 0) t.holderLength = v;
        fillDefaults(t, flute, angle, flutes);
        tools.push_back(t);
    }
    return true;
}

// --- Fusion 360 ------------------------------------------------------------------------------

namespace {

struct FusionType {
    const char* name;
    CamToolType type;
};
constexpr FusionType kFusionTypes[] = {{"flat end mill", CamToolType::Flat},     {"ball end mill", CamToolType::Ball},
                                       {"bull nose end mill", CamToolType::BullNose}, {"radius mill", CamToolType::BullNose},
                                       {"face mill", CamToolType::FaceMill},    {"drill", CamToolType::Drill},
                                       {"spot drill", CamToolType::Spot},       {"center drill", CamToolType::Spot},
                                       {"chamfer mill", CamToolType::Chamfer},  {"counter sink", CamToolType::Chamfer},
                                       {"tap right hand", CamToolType::Tap}};

} // namespace

std::string toolsToFusion(const std::vector<CamTool>& tools) {
    Json data = Json::array();
    for (const CamTool& t : tools) {
        const char* type = "flat end mill";
        for (const FusionType& f : kFusionTypes)
            if (f.type == t.type) {
                type = f.name;
                break;
            }
        Json geo = Json::object();
        geo.set("DC", t.diameter).set("NOF", t.flutes).set("LCF", t.fluteLength).set("RE", t.type == CamToolType::Ball ? t.radius() : t.cornerRadius);
        if (t.type == CamToolType::Drill || t.type == CamToolType::Spot) geo.set("SIG", t.tipAngle);
        if (t.type == CamToolType::Chamfer) geo.set("TA", t.tipAngle / 2);
        if (t.type == CamToolType::Tap) geo.set("TP", t.pitch);
        if (t.stickout > 0) geo.set("LB", t.stickout);
        Json o = Json::object();
        o.set("type", type).set("unit", "millimeters").set("description", t.name).set("vendor", t.vendor).set("product-id", t.productId);
        o.set("BMC", t.hss ? "hss" : "carbide").set("geometry", geo);
        o.set("post-process", Json::object().set("number", t.number));
        if (t.vc > 0 || t.fz > 0) {
            Json preset = Json::object();
            preset.set("name", "Default");
            if (t.vc > 0) preset.set("v_c", t.vc);
            if (t.fz > 0) preset.set(t.type == CamToolType::Drill || t.type == CamToolType::Spot ? "f_n" : "f_z", t.fz);
            o.set("start-values", Json::object().set("presets", Json(Json::Array{preset})));
        }
        data.push(o);
    }
    return Json::object().set("data", data).set("version", 2).dump();
}

bool toolsFromFusion(const std::string& text, std::vector<CamTool>& tools, std::string* error, std::vector<std::string>* notes) {
    std::string why;
    const auto doc = Json::parse(text, &why);
    if (!doc || !(*doc)["data"].isArray()) {
        if (error) *error = "Not a Fusion 360 tool library: " + (why.empty() ? std::string("no \"data\" list") : why);
        return false;
    }
    int next = 1;
    for (const CamTool& t : tools) next = std::max(next, t.number + 1);
    int index = 0;
    for (const Json& o : (*doc)["data"].items()) {
        ++index;
        const std::string type = lowerOf(o["type"].asString(""));
        CamTool t;
        bool known = false;
        for (const FusionType& f : kFusionTypes)
            if (type == f.name) t.type = f.type, known = true;
        if (!known) {
            if (type != "holder" && type != "probe" && notes) notes->push_back("tool " + std::to_string(index) + ": '" + type + "' left out");
            continue;
        }
        const bool inch = lowerOf(o["unit"].asString("millimeters")).find("inch") != std::string::npos;
        const double k = inch ? 25.4 : 1.0;
        const Json& g = o["geometry"];
        t.diameter = g["DC"].asNumber(0) * k;
        if (!(t.diameter > 0.05) || t.diameter > 300) {
            if (notes) notes->push_back("tool " + std::to_string(index) + ": no usable diameter, left out");
            continue;
        }
        const bool flutes = g["NOF"].isNumber() && g["NOF"].asNumber() >= 1;
        if (flutes) t.flutes = std::clamp(g["NOF"].asInt(3), 1, 20);
        const bool flute = g["LCF"].asNumber(0) > 0;
        if (flute) t.fluteLength = g["LCF"].asNumber() * k;
        if (t.type != CamToolType::Ball) t.cornerRadius = g["RE"].asNumber(0) * k;
        bool angle = false;
        if (g["SIG"].asNumber(0) > 0) t.tipAngle = g["SIG"].asNumber(), angle = true;
        if (t.type == CamToolType::Chamfer && g["TA"].asNumber(0) > 0) t.tipAngle = 2 * g["TA"].asNumber(), angle = true;
        if (g["TP"].asNumber(0) > 0) t.pitch = g["TP"].asNumber() * k;
        if (g["LB"].asNumber(0) > 0) t.stickout = g["LB"].asNumber() * k;
        t.hss = lowerOf(o["BMC"].asString("carbide")) == "hss";
        t.name = o["description"].asString("");
        t.vendor = o["vendor"].asString("");
        t.productId = o["product-id"].asString("");
        const int n = o["post-process"]["number"].asInt(0);
        t.number = n >= 1 && n <= 999 ? n : next;
        next = std::max(next, t.number + 1);
        // The first preset's cutting data (feet per minute and inches in an inch library).
        const auto& presets = o["start-values"]["presets"].items();
        if (!presets.empty()) {
            const Json& p = presets.front();
            if (p["v_c"].asNumber(0) > 0) t.vc = p["v_c"].asNumber() * (inch ? 0.3048 : 1.0);
            const bool drill = t.type == CamToolType::Drill || t.type == CamToolType::Spot;
            const double fz = drill ? p["f_n"].asNumber(0) : p["f_z"].asNumber(0);
            if (fz > 0) t.fz = fz * k;
        }
        fillDefaults(t, flute, angle, flutes);
        tools.push_back(t);
    }
    return true;
}

bool loadToolFile(const std::string& path, std::vector<CamTool>& tools, std::string* error, std::vector<std::string>* notes, ToolFileFormat format) {
    std::string text;
    if (!readText(path, text, error)) return false;
    if (format == ToolFileFormat::Auto) {
        const std::string ext = lowerOf(fs::path(path).extension().string());
        const auto first = text.find_first_not_of(" \t\r\n\xEF\xBB\xBF");
        if (ext == ".csv" || ext == ".txt" || (first != std::string::npos && text[first] != '{')) format = ToolFileFormat::Csv;
        else if (text.find("\"sicdm-tools\"") != std::string::npos) format = ToolFileFormat::Native;
        else format = ToolFileFormat::Fusion;
    }
    if (format == ToolFileFormat::Csv) return toolsFromCsv(text, tools, error, notes);
    if (format == ToolFileFormat::Fusion) return toolsFromFusion(text, tools, error, notes);
    std::string why;
    const auto doc = Json::parse(text, &why);
    if (!doc || (*doc)["format"].asString("") != "sicdm-tools") {
        if (error) *error = "Not an SI CDM tool library" + (why.empty() ? std::string() : ": " + why);
        return false;
    }
    for (const Json& o : (*doc)["tools"].items()) tools.push_back(camToolFromJson(o));
    return true;
}

bool saveToolFile(const std::string& path, const std::vector<CamTool>& tools, std::string* error, ToolFileFormat format) {
    if (format == ToolFileFormat::Auto) format = lowerOf(fs::path(path).extension().string()) == ".csv" ? ToolFileFormat::Csv : ToolFileFormat::Native;
    std::string data;
    if (format == ToolFileFormat::Csv) data = toolsToCsv(tools);
    else if (format == ToolFileFormat::Fusion) data = toolsToFusion(tools);
    else {
        Json list = Json::array();
        for (const CamTool& t : tools) list.push(camToolToJson(t));
        data = Json::object().set("format", "sicdm-tools").set("version", 1).set("tools", list).dump();
    }
    std::error_code ec;
    const fs::path dir = fs::path(path).parent_path();
    if (!dir.empty()) fs::create_directories(dir, ec);
    return writeFile(path, data, error);
}

} // namespace sicdm
