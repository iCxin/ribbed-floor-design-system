/**
 * ============================================================================
 *  水工钢筋混凝土肋形楼盖辅助设计系统 V1.0 —— 入口
 * ============================================================================
 *  用法：
 *    ribbed_floor_design_system --selftest   运行系统自检（CRC/MD5/协议/FEM/存储/流水线）
 *    ribbed_floor_design_system --demo       运行典型双向板设计演示
 *    ribbed_floor_design_system --one-way    运行单向板设计演示
 *    ribbed_floor_design_system --frame <hex文件>  处理外部十六进制帧
 *    ribbed_floor_design_system --hex <十六进制字符串>  处理外部十六进制帧
 * ============================================================================
 */
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <string>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif

#include "system_controller.h"
#include "struct_calc.h"
#include "protocol.h"
#include "web_server.h"

using namespace rfds;

/* ------------------------------------------------------------------------ */
/* 测试辅助                                                                    */
/* ------------------------------------------------------------------------ */
static int g_checks_total = 0;
static int g_checks_failed = 0;

static void report_check(bool ok, const char* name, const std::string& detail = "") {
    ++g_checks_total;
    std::printf("[%s] %s%s%s\n", ok ? "PASS" : "FAIL", name,
                detail.empty() ? "" : "  -- ", detail.c_str());
    if (!ok) ++g_checks_failed;
}

/* 构造 FC01 楼盖参数帧 */
static std::vector<uint8_t> make_slab_frame(uint8_t addr, int grade,
                                            float span_mm, float width_mm,
                                            float thick_mm, float ratio,
                                            uint8_t slab_type, float dead_kPa,
                                            float live_kPa, uint8_t dur) {
    std::vector<uint8_t> payload;
    payload.push_back(static_cast<uint8_t>(grade));
    put_f32_le(payload, span_mm);
    put_f32_le(payload, width_mm);
    put_f32_le(payload, thick_mm);
    put_f32_le(payload, ratio);
    payload.push_back(slab_type);
    put_f32_le(payload, dead_kPa);
    put_f32_le(payload, live_kPa);
    payload.push_back(dur);
    payload.push_back(0x00);   /* 保留 */
    return build_frame(addr, FC_CONCRETE_SLAB_PARAM, payload);
}

static std::string to_hex(const std::vector<uint8_t>& data) {
    static const char* digits = "0123456789ABCDEF";
    std::string out;
    out.reserve(data.size() * 2);
    for (uint8_t b : data) {
        out.push_back(digits[b >> 4]);
        out.push_back(digits[b & 0xF]);
    }
    return out;
}

/* ------------------------------------------------------------------------ */
/* 自检 1：校验算法黄金向量                                                    */
/* ------------------------------------------------------------------------ */
static void selftest_crypto() {
    std::printf("\n===== 1. 校验算法自检 =====\n");
    const uint16_t c16 = crc16(reinterpret_cast<const uint8_t*>("123456789"), 9);
    report_check(c16 == 0x29B1, "CRC16-XMODEM(初值0xFFFF) 黄金向量",
                 "123456789 -> " + to_hex({static_cast<uint8_t>(c16 >> 8),
                                           static_cast<uint8_t>(c16 & 0xFF)}) +
                 " (期望 29B1)");

    const uint32_t c32 = crc32(reinterpret_cast<const uint8_t*>("123456789"), 9);
    report_check(c32 == 0xCBF43926u, "CRC32-IEEE 黄金向量",
                 "123456789 -> " + std::to_string(c32));

    Md5Digest d;
    md5_compute(std::string("abc"), d);
    std::string hex;
    for (uint8_t b : d.bytes) {
        char buf[3];
        std::snprintf(buf, sizeof(buf), "%02x", b);
        hex += buf;
    }
    report_check(hex == "900150983cd24fb0d6963f7d28e17f72",
                 "MD5 黄金向量", "abc -> " + hex);
}

/* ------------------------------------------------------------------------ */
/* 自检 2：帧协议（组帧 -> 帧同步 -> 提取）                                    */
/* ------------------------------------------------------------------------ */
static void selftest_protocol() {
    std::printf("\n===== 2. 帧协议自检 =====\n");
    const std::vector<uint8_t> payload = {0x1E, 0x10, 0x20};
    const std::vector<uint8_t> frame = build_frame(0x02, FC_CONCRETE_SLAB_PARAM, payload);

    size_t s = 0, e = 0;
    report_check(synchronize_frame(frame, s, e), "帧同步定位");
    report_check(s == 0 && e == frame.size(), "帧边界完整",
                 "start=" + std::to_string(s) + " end=" + std::to_string(e));

    RawFrame raw;
    ParameterError err;
    report_check(extract_and_validate_packet(frame, s, e, raw, err),
                 "CRC 校验与载荷提取", err.message);
    report_check(raw.device_address == 0x02 &&
                 raw.function_code == FC_CONCRETE_SLAB_PARAM &&
                 raw.payload == payload, "地址/功能码/载荷一致性");

    /* 前置噪声流中提取 */
    std::vector<uint8_t> noisy = {0x00, 0xFF, 0x33};
    noisy.insert(noisy.end(), frame.begin(), frame.end());
    noisy.push_back(0x77);
    report_check(synchronize_frame(noisy, s, e) &&
                 extract_and_validate_packet(noisy, s, e, raw, err),
                 "含噪声字节流的帧恢复");

    /* 篡改载荷 -> CRC 失败 */
    std::vector<uint8_t> corrupted = frame;
    corrupted[kFrameHeaderSize + 5] ^= 0x01;
    report_check(synchronize_frame(corrupted, s, e) &&
                 !extract_and_validate_packet(corrupted, s, e, raw, err) &&
                 err.code == ParameterErrorCode::CRC_ERROR,
                 "数据畸变检测（CRC16）", err.message);
}

/* ------------------------------------------------------------------------ */
/* 自检 3：参数解析模块（含非法参数用例）                                      */
/* ------------------------------------------------------------------------ */
static void selftest_param_parser() {
    std::printf("\n===== 3. 参数解析模块自检 =====\n");
    ParameterParser parser;

    /* 合法双向板 */
    auto good = make_slab_frame(0x02, 30, 6000.0f, 4500.0f, 120.0f,
                                0.005f, 1, 3.0f, 2.5f, 3);
    DesignParam param;
    report_check(parser.Parse_Param(good.data(),
                                    static_cast<uint16_t>(good.size()), &param),
                 "合法双向板参数解析");
    report_check(std::fabs(param.slab_span_m - 6.0) < 1e-6 &&
                 std::fabs(param.slab_width_m - 4.5) < 1e-6 &&
                 param.fc_design_MPa == 16.7 && param.ft_design_MPa == 1.45,
                 "单位换算与强度查表",
                 "span=" + std::to_string(param.slab_span_m) +
                 "m fc=" + std::to_string(param.fc_design_MPa) + "MPa");

    /* CRC 畸变 */
    auto bad_crc = good;
    bad_crc[bad_crc.size() - 4] ^= 0xFF;
    bool rejected = !parser.Parse_Param(bad_crc.data(),
                                        static_cast<uint16_t>(bad_crc.size()), &param);
    report_check(rejected && parser.last_error().code == ParameterErrorCode::CRC_ERROR,
                 "CRC 畸变拒绝", parser.last_error().message);

    /* 强度等级越界（C15） */
    auto bad_grade = make_slab_frame(0x02, 15, 6000.0f, 4500.0f, 120.0f,
                                     0.005f, 1, 3.0f, 2.5f, 3);
    rejected = !parser.Parse_Param(bad_grade.data(),
                                   static_cast<uint16_t>(bad_grade.size()), &param);
    report_check(rejected &&
                 parser.last_error().code == ParameterErrorCode::STRENGTH_GRADE_OUT_OF_RANGE,
                 "强度等级越界拒绝（C15）", parser.last_error().message);

    /* 双向板长宽比冲突（声明双向但比例 4:1） */
    auto bad_ratio = make_slab_frame(0x02, 30, 6000.0f, 1500.0f, 120.0f,
                                     0.005f, 1, 3.0f, 2.5f, 3);
    rejected = !parser.Parse_Param(bad_ratio.data(),
                                   static_cast<uint16_t>(bad_ratio.size()), &param);
    report_check(rejected &&
                 parser.last_error().code == ParameterErrorCode::ASPECT_RATIO_INVALID,
                 "双向板长宽比逻辑冲突拒绝", parser.last_error().message);

    /* 配筋率过低 */
    auto bad_rebar = make_slab_frame(0x02, 30, 6000.0f, 4500.0f, 120.0f,
                                     0.0005f, 1, 3.0f, 2.5f, 3);
    rejected = !parser.Parse_Param(bad_rebar.data(),
                                   static_cast<uint16_t>(bad_rebar.size()), &param);
    report_check(rejected &&
                 parser.last_error().code == ParameterErrorCode::REBAR_RATIO_OUT_OF_RANGE,
                 "配筋率低于限值拒绝", parser.last_error().message);

    /* 板厚过薄 */
    auto bad_thick = make_slab_frame(0x02, 30, 6000.0f, 4500.0f, 50.0f,
                                     0.005f, 1, 3.0f, 2.5f, 3);
    rejected = !parser.Parse_Param(bad_thick.data(),
                                   static_cast<uint16_t>(bad_thick.size()), &param);
    report_check(rejected &&
                 parser.last_error().code == ParameterErrorCode::THICKNESS_OUT_OF_RANGE,
                 "板厚低于规范最小值拒绝", parser.last_error().message);

    /* 帧丢失 */
    const std::vector<uint8_t> no_frame = {0xAA, 0x55, 0x01, 0x02};
    rejected = !parser.Parse_Param(no_frame.data(),
                                   static_cast<uint16_t>(no_frame.size()), &param);
    report_check(rejected && parser.last_error().code == ParameterErrorCode::FRAME_LOSS,
                 "帧丢失检测", parser.last_error().message);
}

/* ------------------------------------------------------------------------ */
/* 自检 4：结构计算模块（FEM 对比 Timoshenko 薄板解析解）                      */
/* ------------------------------------------------------------------------ */
static void selftest_fem() {
    std::printf("\n===== 4. 结构计算模块自检（四边简支方板解析解对比） =====\n");
    /* 四边简支方板 a=4m，h=150mm，C30，均布荷载 10kPa
     * 解析解：w_center = 0.00406 · q·a⁴ / D（Timoshenko 板壳理论） */
    DesignParam param;
    param.slab_span_m = 4.0;
    param.slab_width_m = 4.0;
    param.slab_thickness_m = 0.15;
    param.concrete_grade = 30;
    param.fc_design_MPa = 16.7;
    param.ft_design_MPa = 1.45;
    param.rebar_ratio = 0.005;
    param.slab_type = SLAB_TWO_WAY;
    param.dead_load_kPa = 6.0;
    param.live_load_kPa = 4.0;
    param.load_duration_class = 3;

    const double E = 3.00e10, nu = 0.2, h = 0.15;
    const double D = E * h * h * h / (12.0 * (1.0 - nu * nu));
    const double q = 10.0e3, a = 4.0;
    const double w_analytic = 0.00406 * q * a * a * a * a / D;

    StructuralCalculator calc;
    CalcResult result;
    ParameterError err;
    const bool ok = calc.perform_structural_analysis(param, result, err);
    report_check(ok, "FEM 求解收敛", err.message);
    if (!ok) return;

    const double w_fem = result.max_node_displacement_m;
    const double rel_err = std::fabs(w_fem - w_analytic) / w_analytic;
    char buf[160];
    std::snprintf(buf, sizeof(buf),
                  "解析解=%.4fmm FEM=%.4fmm 相对误差=%.2f%%",
                  w_analytic * 1000.0, w_fem * 1000.0, rel_err * 100.0);
    report_check(rel_err < 0.08, "挠度与 Timoshenko 解析解偏差 < 8%", buf);

    /* 跨中弯矩量级核对：简支方板跨中 Mx = 0.0479·q·a²（解析） */
    const double m_analytic = 0.0479 * q * a * a / 1000.0;   /* kN·m/m */
    const double m_fem = result.forces.midspan_moment_x_kNm;
    const double m_err = std::fabs(m_fem - m_analytic) / m_analytic;
    std::snprintf(buf, sizeof(buf),
                  "解析解=%.2f FEM=%.2f kN·m/m 相对误差=%.2f%%",
                  m_analytic, m_fem, m_err * 100.0);
    report_check(m_err < 0.15, "跨中弯矩与解析解偏差 < 15%", buf);

    /* 支座负弯矩接近 0（简支边） */
    report_check(result.forces.support_moment_x_kNm <
                 0.15 * result.forces.midspan_moment_x_kNm,
                 "简支边支座负弯矩近似为零",
                 "M-=" + std::to_string(result.forces.support_moment_x_kNm));

    /* 挠度限值检查逻辑 */
    report_check(result.deflection_check.allowable_deflection_m > 0.0 &&
                 result.deflection_check.message.size() > 0,
                 "挠度验算结论生成",
                 result.deflection_check.message + " (长期挠度 " +
                 std::to_string(result.deflection_check.long_term_deflection_m * 1000.0) +
                 "mm / 允许 " +
                 std::to_string(result.deflection_check.allowable_deflection_m * 1000.0) + "mm)");
}

/* ------------------------------------------------------------------------ */
/* 自检 5：数据存储模块                                                        */
/* ------------------------------------------------------------------------ */
static void selftest_storage() {
    std::printf("\n===== 5. 数据存储模块自检 =====\n");
    hal_flash_reset();
    StorageManagementModule store;
    store.initialize();

    /* 写入并回读 */
    std::vector<uint8_t> blob(200);
    for (size_t i = 0; i < blob.size(); ++i) blob[i] = static_cast<uint8_t>(i * 7);
    report_check(store.Store_Data(STORE_DESIGN_PARAM, blob.data(), 200, 1001),
                 "记录写入（原子事务）");
    std::vector<uint8_t> out;
    StoreType t;
    report_check(store.query_record(1001, out, &t) && out == blob &&
                 t == STORE_DESIGN_PARAM, "按设计编号回读一致性");

    /* 哈希索引查询 */
    Md5Digest d;
    md5_compute(blob.data(), blob.size(), d);
    uint32_t hash = 0;
    std::memcpy(&hash, d.bytes, 4);
    FileMetadata meta;
    report_check(store.fat().find_by_hash(hash ? hash : 1, meta),
                 "MD5 哈希索引检索",
                 "index=0x" + to_hex({d.bytes[0], d.bytes[1], d.bytes[2], d.bytes[3]}));

    /* 同编号覆盖 */
    std::vector<uint8_t> blob2(150, 0x5A);
    report_check(store.Store_Data(STORE_OPT_RESULT, blob2.data(), 150, 1001),
                 "同设计编号循环覆盖写入");
    report_check(store.query_record(1001, out) && out == blob2,
                 "覆盖后读到最新记录");

    /* 容量压力与最旧记录淘汰（32 扇区，写入 64 条触发覆盖） */
    std::vector<uint8_t> small(32, 0x11);
    bool all_ok = true;
    for (uint32_t i = 0; i < 64; ++i) {
        if (!store.Store_Data(STORE_CALC_RESULT, small.data(), 32, 2000 + i)) {
            all_ok = false;
            break;
        }
    }
    report_check(all_ok, "连续写入 64 条（含自动 GC/覆盖）");
    report_check(store.record_count() <= 40, "记录数受容量约束",
                 "count=" + std::to_string(store.record_count()) +
                 " free=" + std::to_string(store.free_ratio() * 100.0) + "%");
    report_check(store.query_record(2000, out) == false,
                 "最旧记录已按时间戳淘汰");
    report_check(store.query_record(2063, out) && out == small,
                 "最新记录完整保留");

    /* 磨损均衡：32 次写入后各扇区擦写次数应尽量均匀 */
    uint32_t max_wear = 0, min_wear = 0xFFFFFFFFu;
    for (size_t s = 0; s < kFlashSectorCount; ++s) {
        const uint32_t w = hal_flash_get_erase_count(s);
        max_wear = std::max(max_wear, w);
        min_wear = std::min(min_wear, w);
    }
    report_check(max_wear - min_wear <= 2, "磨损均衡（擦写次数极差 <= 2）",
                 "max=" + std::to_string(max_wear) + " min=" +
                 std::to_string(min_wear));
}

/* ------------------------------------------------------------------------ */
/* 自检 6：端到端流水线                                                        */
/* ------------------------------------------------------------------------ */
static void selftest_pipeline() {
    std::printf("\n===== 6. 端到端流水线自检 =====\n");
    EmbeddedSystemController controller;
    controller.initialize();

    auto frame = make_slab_frame(0x02, 35, 6600.0f, 4800.0f, 130.0f,
                                 0.004f, 1, 4.5f, 3.5f, 3);
    PipelineReport report;
    const bool ok = controller.process_one_design(
        frame.data(), static_cast<uint16_t>(frame.size()), report);
    report_check(ok, "参数->计算->优化->封装 全链路",
                 report.error.message +
                 (ok ? "" : "（错误码 0x" + to_hex({static_cast<uint8_t>(report.error.code)}) + "）"));
    if (!ok) return;

    report_check(report.output_frame.size() >= kMinPacketSize &&
                 report.output_frame.size() <= kMaxOutputFrameSize,
                 "输出帧长度合规 (<=128B)",
                 std::to_string(report.output_frame.size()) + "B");

    /* 输出帧可被协议层回验（自反性） */
    size_t s = 0, e = 0;
    RawFrame raw;
    ParameterError err;
    report_check(synchronize_frame(report.output_frame, s, e) &&
                 extract_and_validate_packet(report.output_frame, s, e, raw, err) &&
                 raw.function_code == FC_RESULT_PACKAGE,
                 "输出帧 CRC 自反校验");

    report_check(report.opt.safety_factor >= 1.0, "承载力安全储备 >= 1.0",
                 "Mu/(K·M) = " + std::to_string(report.opt.safety_factor));
    report_check(report.opt.bottom_x.bar_diameter_mm >= 8 &&
                 report.opt.bottom_x.bar_spacing_mm >= 50 &&
                 report.opt.bottom_x.bar_spacing_mm <= 200,
                 "板底受力筋构造合规",
                 "D" + std::to_string(report.opt.bottom_x.bar_diameter_mm) +
                 "@" + std::to_string(report.opt.bottom_x.bar_spacing_mm));

    /* 存储链路：设计参数与结果帧均已落盘 */
    std::vector<uint8_t> stored;
    report_check(controller.storage().query_record(report.design_number, stored),
                 "设计记录已存档（唯一编号可追溯）",
                 "design_no=" + std::to_string(report.design_number));

    /* 系统监控 */
    const SystemHealthReport hr = controller.run_monitor_task();
    report_check(!hr.abnormal, "系统监控任务健康", hr.note);

    std::printf("\n输出帧十六进制（%zu 字节）：\n  %s\n",
                report.output_frame.size(), to_hex(report.output_frame).c_str());
}

/* ------------------------------------------------------------------------ */
/* 演示：典型工程设计                                                          */
/* ------------------------------------------------------------------------ */
static void print_scheme(const char* name, const RebarScheme& s) {
    if (s.bar_diameter_mm == 0) {
        std::printf("  %-14s : 无\n", name);
        return;
    }
    std::printf("  %-14s : D%d @ %4.0f mm   As=%6.1f mm2/m   Mu=%6.2f kN·m/m   "
                "sigma_s=%5.1f MPa   w=%5.3f mm\n",
                name, s.bar_diameter_mm, s.bar_spacing_mm, s.area_mm2_per_m,
                s.moment_capacity_kNm, s.steel_stress_MPa, s.crack_width_mm);
}

static void run_demo(bool one_way) {
    std::printf("\n================ %s设计演示 ================\n",
                one_way ? "单向板" : "双向板");
    EmbeddedSystemController controller;
    controller.initialize();

    auto frame = one_way
        ? make_slab_frame(0x02, 30, 6000.0f, 1800.0f, 110.0f, 0.006f, 0,
                          3.5f, 2.5f, 3)
        : make_slab_frame(0x02, 30, 6000.0f, 4500.0f, 120.0f, 0.004f, 1,
                          3.0f, 2.5f, 3);

    PipelineReport report;
    if (!controller.process_one_design(frame.data(),
                                       static_cast<uint16_t>(frame.size()),
                                       report)) {
        std::printf("设计失败：%s（错误码 0x%02X）\n",
                    report.error.message.c_str(),
                    static_cast<unsigned>(report.error.code));
        return;
    }

    const PipelineReport& r = report;
    std::printf("\n--- 设计输入 ---\n");
    std::printf("  设计编号 : %u（已存档可追溯）\n", r.design_number);
    std::printf("  混凝土   : C%d  (fc=%g MPa, ft=%g MPa)\n",
                r.param.concrete_grade, r.param.fc_design_MPa,
                r.param.ft_design_MPa);
    std::printf("  几何     : %.2f m x %.2f m，板厚 %.0f mm，%s\n",
                r.param.slab_span_m, r.param.slab_width_m,
                r.param.slab_thickness_m * 1000.0,
                r.param.slab_type == SLAB_TWO_WAY ? "双向板" : "单向板");
    std::printf("  荷载     : 恒载 %.1f kPa + 活载 %.1f kPa\n",
                r.param.dead_load_kPa, r.param.live_load_kPa);

    std::printf("\n--- 结构计算（ACM 薄板单元 %dx%d 网格，%d 自由度）---\n",
                r.calc.num_nodes_x - 1, r.calc.num_nodes_y - 1,
                r.calc.total_dofs);
    std::printf("  跨中弯矩 : X 向 %.2f kN·m/m，Y 向 %.2f kN·m/m\n",
                r.calc.forces.midspan_moment_x_kNm,
                r.calc.forces.midspan_moment_y_kNm);
    std::printf("  支座弯矩 : X 向 %.2f kN·m/m，Y 向 %.2f kN·m/m\n",
                r.calc.forces.support_moment_x_kNm,
                r.calc.forces.support_moment_y_kNm);
    std::printf("  支座剪力 : %.2f kN/m\n", r.calc.forces.max_shear_kN);
    std::printf("  挠度验算 : 短期 %.2f mm x 长期系数 %.2f = %.2f mm，"
                "允许 %.2f mm  [%s]\n",
                r.calc.deflection_check.short_term_deflection_m * 1000.0,
                r.calc.deflection_check.deflection_increase_factor,
                r.calc.deflection_check.long_term_deflection_m * 1000.0,
                r.calc.deflection_check.allowable_deflection_m * 1000.0,
                r.calc.deflection_check.message.c_str());
    std::printf("  裂缝验算 : 最大 %.3f mm，允许 %.2f mm  [%s]\n",
                r.calc.crack_check.max_crack_width_mm,
                r.calc.crack_check.allowable_crack_width_mm,
                r.calc.crack_check.message.c_str());

    std::printf("\n--- 配筋方案（迭代 %d 次%s）---\n",
                r.opt.iterations_used,
                r.opt.constructive_adjusted ? "，构造修正" : "");
    print_scheme("板底 X 向", r.opt.bottom_x);
    print_scheme("板底 Y 向", r.opt.bottom_y);
    print_scheme("支座 X 顶筋", r.opt.top_x);
    print_scheme("支座 Y 顶筋", r.opt.top_y);
    std::printf("  配筋率   : %.3f%%\n", r.opt.total_rebar_ratio * 100.0);
    std::printf("  钢筋用量 : 约 %.1f kg\n", r.opt.steel_weight_kg);
    std::printf("  安全储备 : Mu/(K·M) = %.2f\n", r.opt.safety_factor);
    std::printf("  结论     : %s\n", r.opt.summary.c_str());

    std::printf("\n--- 输出帧（%zu 字节，已发送至 UART/存储）---\n",
                r.output_frame.size());
    const std::string hex = to_hex(r.output_frame);
    for (size_t i = 0; i < hex.size(); i += 48)
        std::printf("  %s\n", hex.substr(i, 48).c_str());

    const SystemHealthReport hr = controller.run_monitor_task();
    std::printf("\n--- 系统状态 ---\n  主频 %.0f MHz，估算功耗 %.0f mW，%s\n",
                static_cast<double>(hr.current_freq == CpuFreq::FREQ_400MHZ
                                        ? 400.0 : 200.0),
                hr.estimated_power_mw, hr.note.c_str());
}

/* ------------------------------------------------------------------------ */
/* 十六进制帧处理                                                              */
/* ------------------------------------------------------------------------ */
static int run_hex_frame(const std::string& hex) {
    if (hex.size() % 2 != 0) {
        std::fprintf(stderr, "错误：十六进制串长度必须为偶数\n");
        return 2;
    }
    std::vector<uint8_t> bytes;
    for (size_t i = 0; i < hex.size(); i += 2) {
        bytes.push_back(static_cast<uint8_t>(
            std::strtoul(hex.substr(i, 2).c_str(), nullptr, 16)));
    }
    EmbeddedSystemController controller;
    controller.initialize();
    PipelineReport report;
    if (!controller.process_one_design(bytes.data(),
                                       static_cast<uint16_t>(bytes.size()),
                                       report)) {
        std::fprintf(stderr, "处理失败：%s（错误码 0x%02X）\n",
                     report.error.message.c_str(),
                     static_cast<unsigned>(report.error.code));
        return 1;
    }
    std::printf("%s\n", to_hex(report.output_frame).c_str());
    return 0;
}

/* ------------------------------------------------------------------------ */
/* main                                                                        */
/* ------------------------------------------------------------------------ */
int main(int argc, char** argv) {
#ifdef _WIN32
    SetConsoleOutputCP(65001);   /* Windows 控制台 UTF-8 输出 */
#endif

    std::printf("%s V%s（FreeRTOS/STM32H743ZI2 目标环境 PC 模拟内核）\n",
                kSystemName, kSystemVersion);

    std::string hex_arg;
    uint16_t web_port = 8080;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--web") {
            if (i + 1 < argc && argv[i + 1][0] >= '0' && argv[i + 1][0] <= '9') {
                web_port = static_cast<uint16_t>(std::strtoul(argv[++i], nullptr, 10));
            }
            WebServer server(web_port);
            std::string err;
            if (!server.run(err)) {
                std::fprintf(stderr, "[web] 启动失败：%s\n", err.c_str());
                return 1;
            }
            return 0;
        }
        if (arg == "--selftest") {
            selftest_crypto();
            selftest_protocol();
            selftest_param_parser();
            selftest_fem();
            selftest_storage();
            selftest_pipeline();
            std::printf("\n===== 自检汇总：%d/%d 通过 =====\n",
                        g_checks_total - g_checks_failed, g_checks_total);
            return g_checks_failed == 0 ? 0 : 1;
        }
        if (arg == "--demo")  { run_demo(false); return 0; }
        if (arg == "--one-way") { run_demo(true); return 0; }
        if (arg == "--hex" && i + 1 < argc) { hex_arg = argv[++i]; continue; }
        if (arg == "--frame" && i + 1 < argc) {
            FILE* f = std::fopen(argv[++i], "rb");
            if (!f) { std::fprintf(stderr, "错误：无法打开帧文件\n"); return 2; }
            char buf[4096];
            size_t n = std::fread(buf, 1, sizeof(buf) - 1, f);
            std::fclose(f);
            buf[n] = '\0';
            for (size_t k = 0; k < n; ++k)
                if (buf[k] == '\r' || buf[k] == '\n' || buf[k] == ' ') buf[k] = '\0';
            hex_arg = buf;   /* 取第一段 */
            break;
        }
        std::fprintf(stderr,
                     "用法：%s --selftest | --demo | --one-way | --web [端口] | "
                     "--hex <hex> | --frame <file>\n", argv[0]);
        return 2;
    }
    if (!hex_arg.empty()) return run_hex_frame(hex_arg);

    /* 默认：演示模式 */
    run_demo(false);
    return 0;
}
