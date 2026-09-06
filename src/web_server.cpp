/**
 * web_server.cpp —— Web 端服务实现（本地 HTTP 服务器 + JSON API）
 *
 * 结构化参数经 /api/design 组装为协议帧后走与硬件接口完全一致的
 * process_one_design 流水线（参数解析 -> 结构计算 -> 配筋优化 ->
 * 结果封装 -> 数据存储），保证 Web 端与嵌入式端计算结果逐位一致。
 */
#include "web_server.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <vector>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#ifdef _MSC_VER
#pragma comment(lib, "ws2_32.lib")
#endif
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
using SOCKET = int;
#define closesocket close
#define INVALID_SOCKET (-1)
#endif

#include "system_controller.h"
#include "protocol.h"

namespace rfds {

/* ========================================================================== */
/* 极简 JSON：扁平对象解析（字符串/数字/布尔）与字符串转义输出                  */
/* ========================================================================== */
namespace minijson {

/* 解析扁平 JSON 对象；值为字符串/数字/布尔，统一转为字符串 */
static bool parse_object(const std::string& text,
                         std::map<std::string, std::string>& out) {
    size_t i = 0;
    const size_t n = text.size();
    auto skip_ws = [&]() { while (i < n && (text[i] == ' ' || text[i] == '\t' ||
                                            text[i] == '\r' || text[i] == '\n')) ++i; };
    skip_ws();
    if (i >= n || text[i] != '{') return false;
    ++i;
    skip_ws();
    if (i < n && text[i] == '}') return true;
    while (i < n) {
        skip_ws();
        if (i >= n || text[i] != '"') return false;
        ++i;
        std::string key;
        while (i < n && text[i] != '"') {
            if (text[i] == '\\' && i + 1 < n) { ++i; }
            key += text[i++];
        }
        ++i;   /* 闭引号 */
        skip_ws();
        if (i >= n || text[i] != ':') return false;
        ++i;
        skip_ws();
        std::string value;
        if (i < n && text[i] == '"') {
            ++i;
            while (i < n && text[i] != '"') {
                if (text[i] == '\\' && i + 1 < n) {
                    ++i;
                    switch (text[i]) {
                        case 'n': value += '\n'; break;
                        case 't': value += '\t'; break;
                        case 'r': value += '\r'; break;
                        default:  value += text[i]; break;
                    }
                    ++i;
                } else {
                    value += text[i++];
                }
            }
            ++i;   /* 闭引号 */
        } else {
            while (i < n && text[i] != ',' && text[i] != '}' &&
                   text[i] != ' ' && text[i] != '\r' && text[i] != '\n') {
                value += text[i++];
            }
        }
        out[key] = value;
        skip_ws();
        if (i < n && text[i] == ',') { ++i; continue; }
        if (i < n && text[i] == '}') return true;
        if (i >= n) return false;
        return false;
    }
    return false;
}

static std::string escape(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 8);
    for (unsigned char c : s) {
        switch (c) {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (c < 0x20) {   /* 其余控制字符转 \u00XX */
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04X", c);
                    out += buf;
                } else {
                    out += static_cast<char>(c);
                }
        }
    }
    return out;
}

/* 数值输出：非整数保留 4 位有效小数以内，去掉多余 0 */
static std::string number(double v) {
    if (!std::isfinite(v)) return "null";
    if (std::fabs(v) < 1e-12) v = 0.0;
    char buf[40];
    std::snprintf(buf, sizeof(buf), "%.6g", v);
    return buf;
}

}  /* namespace minijson */

/* ========================================================================== */
/* HTTP 基础                                                                   */
/* ========================================================================== */
namespace {

struct HttpRequest {
    std::string method;
    std::string path;
    std::string body;
};

static bool recv_all(SOCKET fd, HttpRequest& req) {
    std::string raw;
    char buf[4096];
    const size_t kMaxRequest = 1 << 20;   /* 1MB 上限 */
    size_t header_end = std::string::npos;
    size_t content_len = 0;
    while (raw.size() < kMaxRequest) {
        int got = recv(fd, buf, sizeof(buf), 0);
        if (got <= 0) break;
        raw.append(buf, static_cast<size_t>(got));
        if (header_end == std::string::npos) {
            header_end = raw.find("\r\n\r\n");
            if (header_end != std::string::npos) {
                /* 解析 Content-Length */
                const std::string headers = raw.substr(0, header_end);
                size_t pos = 0;
                while ((pos = headers.find("Content-Length:", pos)) != std::string::npos ||
                       (pos = headers.find("content-length:", pos)) != std::string::npos) {
                    content_len = static_cast<size_t>(std::strtoul(
                        headers.c_str() + pos + 15, nullptr, 10));
                    break;
                }
            }
        }
        if (header_end != std::string::npos &&
            raw.size() >= header_end + 4 + content_len) break;   /* 收齐 */
    }
    if (raw.empty()) return false;

    const size_t line_end = raw.find("\r\n");
    std::string request_line = line_end == std::string::npos ? raw : raw.substr(0, line_end);
    const size_t sp1 = request_line.find(' ');
    const size_t sp2 = request_line.find(' ', sp1 + 1);
    if (sp1 == std::string::npos || sp2 == std::string::npos) return false;
    req.method = request_line.substr(0, sp1);
    req.path = request_line.substr(sp1 + 1, sp2 - sp1 - 1);
    const size_t q = req.path.find('?');
    if (q != std::string::npos) req.path = req.path.substr(0, q);

    if (header_end != std::string::npos &&
        raw.size() >= header_end + 4 + content_len) {
        req.body = raw.substr(header_end + 4, content_len);
    }
    return true;
}

static void send_response(SOCKET fd, int status, const std::string& content_type,
                          const std::string& body) {
    const char* status_text = status == 200 ? "OK" :
                              status == 400 ? "Bad Request" :
                              status == 404 ? "Not Found" :
                              status == 405 ? "Method Not Allowed" : "Error";
    std::ostringstream head;
    head << "HTTP/1.1 " << status << " " << status_text << "\r\n"
         << "Content-Type: " << content_type << "\r\n"
         << "Content-Length: " << body.size() << "\r\n"
         << "Connection: close\r\n"
         << "Cache-Control: no-store\r\n\r\n";
    const std::string head_str = head.str();
    send(fd, head_str.c_str(), static_cast<int>(head_str.size()), 0);
    send(fd, body.c_str(), static_cast<int>(body.size()), 0);
}

static bool read_static_file(const std::string& path, std::string& out) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    std::ostringstream ss;
    ss << f.rdbuf();
    out = ss.str();
    return true;
}

/* ------------------------------------------------------------------------ */
/* 业务处理：结构化参数 -> 协议帧 -> 流水线                                     */
/* ------------------------------------------------------------------------ */
static std::string json_report(PipelineReport& rep) {
    std::ostringstream o;
    const bool ok = rep.stage == DesignStage::PACKAGED;
    o << "{\"ok\":" << (ok ? "true" : "false");
    o << ",\"error\":{\"code\":" << static_cast<int>(rep.error.code)
      << ",\"message\":\"" << minijson::escape(rep.error.message) << "\"}";
    o << ",\"design_number\":" << rep.design_number;
    o << ",\"elapsed_ms\":" << rep.elapsed_ms;

    if (rep.stage == DesignStage::REJECTED || rep.stage == DesignStage::PARSED) {
        /* 参数被拒绝：直接返回错误 */
        o << "}";
        return o.str();
    }

    const DesignParam& p = rep.param;
    const CalcResult& c = rep.calc;
    const OptResult& r = rep.opt;

    o << ",\"input\":{\"grade\":" << p.concrete_grade
      << ",\"fc\":" << minijson::number(p.fc_design_MPa)
      << ",\"ft\":" << minijson::number(p.ft_design_MPa)
      << ",\"span\":" << minijson::number(p.slab_span_m)
      << ",\"width\":" << minijson::number(p.slab_width_m)
      << ",\"thickness\":" << minijson::number(p.slab_thickness_m * 1000.0)
      << ",\"ratio\":" << minijson::number(p.rebar_ratio)
      << ",\"type\":" << p.slab_type
      << ",\"dead\":" << minijson::number(p.dead_load_kPa)
      << ",\"live\":" << minijson::number(p.live_load_kPa)
      << ",\"aspect_ratio\":" << minijson::number(p.slab_span_m / p.slab_width_m)
      << "}";

    o << ",\"calc\":{\"mesh_nx\":" << (c.num_nodes_x - 1)
      << ",\"mesh_ny\":" << (c.num_nodes_y - 1)
      << ",\"dofs\":" << c.total_dofs
      << ",\"mid_x\":" << minijson::number(c.forces.midspan_moment_x_kNm)
      << ",\"sup_x\":" << minijson::number(c.forces.support_moment_x_kNm)
      << ",\"mid_y\":" << minijson::number(c.forces.midspan_moment_y_kNm)
      << ",\"sup_y\":" << minijson::number(c.forces.support_moment_y_kNm)
      << ",\"shear\":" << minijson::number(c.forces.max_shear_kN)
      << ",\"defl_short\":" << minijson::number(c.deflection_check.short_term_deflection_m * 1000.0)
      << ",\"defl_long\":" << minijson::number(c.deflection_check.long_term_deflection_m * 1000.0)
      << ",\"defl_factor\":" << minijson::number(c.deflection_check.deflection_increase_factor)
      << ",\"defl_allow\":" << minijson::number(c.deflection_check.allowable_deflection_m * 1000.0)
      << ",\"defl_passed\":" << (c.deflection_check.passed ? "true" : "false")
      << ",\"crack_max\":" << minijson::number(c.crack_check.max_crack_width_mm)
      << ",\"crack_allow\":" << minijson::number(c.crack_check.allowable_crack_width_mm)
      << ",\"crack_passed\":" << (c.crack_check.passed ? "true" : "false")
      << "}";

    auto scheme_json = [](const char* name, const RebarScheme& s) {
        std::ostringstream so;
        so << "{\"name\":\"" << name << "\",\"d\":" << s.bar_diameter_mm
           << ",\"spacing\":" << minijson::number(s.bar_spacing_mm)
           << ",\"as\":" << minijson::number(s.area_mm2_per_m)
           << ",\"mu\":" << minijson::number(s.moment_capacity_kNm)
           << ",\"sigma\":" << minijson::number(s.steel_stress_MPa)
           << ",\"w\":" << minijson::number(s.crack_width_mm) << "}";
        return so.str();
    };
    o << ",\"opt\":{"
      << "\"schemes\":["
      << scheme_json("bottom_x", r.bottom_x) << ","
      << scheme_json("bottom_y", r.bottom_y) << ","
      << scheme_json("top_x", r.top_x) << ","
      << scheme_json("top_y", r.top_y) << "]"
      << ",\"ratio\":" << minijson::number(r.total_rebar_ratio)
      << ",\"weight\":" << minijson::number(r.steel_weight_kg)
      << ",\"safety\":" << minijson::number(r.safety_factor)
      << ",\"iterations\":" << r.iterations_used
      << ",\"constructive\":" << (r.constructive_adjusted ? "true" : "false")
      << ",\"all_passed\":" << (r.all_checks_passed ? "true" : "false")
      << ",\"summary\":\"" << minijson::escape(r.summary) << "\""
      << "}";

    /* 输出帧十六进制 */
    std::string hex;
    static const char* digits = "0123456789ABCDEF";
    for (uint8_t b : rep.output_frame) {
        hex.push_back(digits[b >> 4]);
        hex.push_back(digits[b & 0xF]);
    }
    o << ",\"frame_hex\":\"" << hex << "\"";
    o << "}";
    return o.str();
}

static std::string handle_api_design(const std::string& body, uint8_t source_addr) {
    std::map<std::string, std::string> kv;
    if (!minijson::parse_object(body, kv)) {
        return "{\"ok\":false,\"error\":{\"code\":400,\"message\":\"JSON 解析失败\"}}";
    }
    auto num = [&](const char* key, double def) {
        auto it = kv.find(key);
        return it == kv.end() ? def : std::strtod(it->second.c_str(), nullptr);
    };

    /* 组装协议帧（与外部设备二进制接口逐字节一致，走同一条解析流水线） */
    std::vector<uint8_t> payload;
    const int grade = static_cast<int>(num("grade", 30));
    payload.push_back(static_cast<uint8_t>(grade));
    put_f32_le(payload, static_cast<float>(num("span_mm", 6000.0)));
    put_f32_le(payload, static_cast<float>(num("width_mm", 4500.0)));
    put_f32_le(payload, static_cast<float>(num("thickness_mm", 120.0)));
    put_f32_le(payload, static_cast<float>(num("ratio", 0.004)));
    payload.push_back(static_cast<uint8_t>(num("type", 1)));
    put_f32_le(payload, static_cast<float>(num("dead_kPa", 3.0)));
    put_f32_le(payload, static_cast<float>(num("live_kPa", 2.5)));
    payload.push_back(static_cast<uint8_t>(num("duration", 3)));
    payload.push_back(0);

    const std::vector<uint8_t> frame =
        build_frame(source_addr, FC_CONCRETE_SLAB_PARAM, payload);

    EmbeddedSystemController controller;
    controller.initialize();
    PipelineReport report;
    controller.process_one_design(frame.data(),
                                  static_cast<uint16_t>(frame.size()), report);
    return json_report(report);
}

static std::string handle_api_frame(const std::string& body) {
    std::map<std::string, std::string> kv;
    if (!minijson::parse_object(body, kv) || kv.find("frame") == kv.end()) {
        return "{\"ok\":false,\"error\":{\"code\":400,\"message\":\"缺少 frame 字段\"}}";
    }
    const std::string& hex = kv["frame"];
    if (hex.size() % 2 != 0) {
        return "{\"ok\":false,\"error\":{\"code\":400,\"message\":\"hex 长度须为偶数\"}}";
    }
    std::vector<uint8_t> bytes;
    for (size_t i = 0; i < hex.size(); i += 2) {
        bytes.push_back(static_cast<uint8_t>(
            std::strtoul(hex.substr(i, 2).c_str(), nullptr, 16)));
    }
    EmbeddedSystemController controller;
    controller.initialize();
    PipelineReport report;
    controller.process_one_design(bytes.data(),
                                  static_cast<uint16_t>(bytes.size()), report);
    return json_report(report);
}

}  /* anonymous namespace */

/* ========================================================================== */
/* WebServer                                                                   */
/* ========================================================================== */
WebServer::WebServer(uint16_t port, std::string static_root)
    : port_(port), static_root_(std::move(static_root)) {}

bool WebServer::run(std::string& err) {
#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        err = "WSAStartup failed";
        return false;
    }
#endif

    const SOCKET listener = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listener == INVALID_SOCKET) {
        err = "socket() failed";
        return false;
    }
    int reuse = 1;
    setsockopt(listener, SOL_SOCKET, SO_REUSEADDR,
               reinterpret_cast<const char*>(&reuse), sizeof(reuse));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);   /* 仅监听本机回环 */
    if (bind(listener, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        err = "端口被占用或绑定失败（127.0.0.1:" + std::to_string(port_) + "）";
        closesocket(listener);
        return false;
    }
    if (listen(listener, 8) != 0) {
        err = "listen() failed";
        closesocket(listener);
        return false;
    }

    std::printf("[web] %s V%s\n", kSystemName, kSystemVersion);
    std::printf("[web] 设计界面: http://127.0.0.1:%u/\n",
                static_cast<unsigned>(port_));
    std::printf("[web] API: GET /api/health | POST /api/design | POST /api/frame\n");
    std::printf("[web] Ctrl+C 停止服务\n");
    std::fflush(stdout);

    for (;;) {
        const SOCKET conn = accept(listener, nullptr, nullptr);
        if (conn == INVALID_SOCKET) continue;

        HttpRequest req;
        if (recv_all(conn, req)) {
            if (req.path == "/" || req.path == "/index.html") {
                std::string page;
                if (read_static_file(static_root_ + "/index.html", page)) {
                    send_response(conn, 200, "text/html; charset=utf-8", page);
                } else {
                    send_response(conn, 404, "text/html; charset=utf-8",
                                  "<h1>404</h1><p>缺少 web/index.html</p>");
                }
            } else if (req.path == "/favicon.ico") {
                send_response(conn, 204, "image/x-icon", "");
            } else if (req.path == "/api/health") {
                std::ostringstream o;
                o << "{\"status\":\"ok\",\"system\":\"" << kSystemName
                  << "\",\"version\":\"" << kSystemVersion << "\",\"port\":" << port_ << "}";
                send_response(conn, 200, "application/json; charset=utf-8", o.str());
            } else if (req.path == "/api/design" && req.method == "POST") {
                send_response(conn, 200, "application/json; charset=utf-8",
                              handle_api_design(req.body, kDeviceAddrMin));
            } else if (req.path == "/api/frame" && req.method == "POST") {
                send_response(conn, 200, "application/json; charset=utf-8",
                              handle_api_frame(req.body));
            } else {
                send_response(conn, 404, "application/json; charset=utf-8",
                              "{\"error\":\"not found\"}");
            }
        }
        closesocket(conn);
    }

#ifdef _WIN32
    WSACleanup();
#endif
    return true;
}

}  /* namespace rfds */
