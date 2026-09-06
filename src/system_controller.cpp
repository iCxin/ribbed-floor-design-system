/**
 * system_controller.cpp —— 系统整合控制器实现
 */
#include "system_controller.h"

#include <cstdio>
#include <cstring>
#include <algorithm>

namespace rfds {

/* OptResult 共享内存紧凑序列化（定长 126B，含尾部 CRC32）
 * 布局：头(16B) + 4 方案 x 10B + 配筋率(2B) + 结论(定长64B) + CRC32(4B) */
constexpr size_t kOptBlobSummarySize = 64;
constexpr size_t kOptBlobSize = 16 + 40 + 2 + kOptBlobSummarySize + 4;   /* 126B */

static void serialize_opt_result(const OptResult& o, std::vector<uint8_t>& out) {
    out.clear();
    out.reserve(kOptBlobSize);
    put_u32_le(out, o.design_number);
    put_u32_le(out, o.timestamp);
    put_u16_le(out, static_cast<uint16_t>(o.safety_factor * 100.0));
    put_u16_le(out, static_cast<uint16_t>(o.steel_weight_kg * 10.0));
    out.push_back(o.all_checks_passed ? 1 : 0);
    out.push_back(static_cast<uint8_t>(std::min(255, o.iterations_used)));
    out.push_back(o.constructive_adjusted ? 1 : 0);
    out.push_back(0);   /* 保留 */
    const RebarScheme* schemes[4] = {&o.bottom_x, &o.bottom_y, &o.top_x, &o.top_y};
    for (int i = 0; i < 4; ++i) {
        out.push_back(static_cast<uint8_t>(schemes[i]->bar_diameter_mm));
        out.push_back(static_cast<uint8_t>(schemes[i]->bar_spacing_mm / 5.0 + 0.5));
        put_u16_le(out, static_cast<uint16_t>(schemes[i]->area_mm2_per_m / 10.0));
        put_u16_le(out, static_cast<uint16_t>(schemes[i]->moment_capacity_kNm * 10.0));
        put_u16_le(out, static_cast<uint16_t>(schemes[i]->steel_stress_MPa / 0.1 + 0.5));
        put_u16_le(out, static_cast<uint16_t>(schemes[i]->crack_width_mm * 1000.0));
    }
    put_u16_le(out, static_cast<uint16_t>(o.total_rebar_ratio * 1.0e5));
    /* 结论文本定长 64B；仅在发生截断时修复 UTF-8 多字节字符边界 */
    std::string s = o.summary;
    if (s.size() > kOptBlobSummarySize) {
        s.resize(kOptBlobSummarySize);
        while (!s.empty() && (s.back() & 0xC0) == 0x80) s.pop_back();
        if (!s.empty() && (s.back() & 0x80) != 0) s.pop_back();   /* 切断字符的首字节 */
    }
    out.insert(out.end(), s.begin(), s.end());
    out.insert(out.end(), kOptBlobSummarySize - s.size(), 0);
    put_u32_le(out, crc32(out));
}

static bool deserialize_opt_result(const uint8_t* p, size_t len, OptResult& o) {
    if (len != kOptBlobSize) return false;
    uint32_t stored_crc = get_u32_le(p + len - 4);
    if (stored_crc != crc32(p, len - 4)) return false;   /* CRC32 校验 */

    size_t off = 0;
    o.design_number = get_u32_le(p + off); off += 4;
    o.timestamp = get_u32_le(p + off); off += 4;
    o.safety_factor = get_u16_le(p + off) / 100.0; off += 2;
    o.steel_weight_kg = get_u16_le(p + off) / 10.0; off += 2;
    o.all_checks_passed = p[off++] == 1;
    o.iterations_used = p[off++];
    o.constructive_adjusted = p[off++] == 1;
    off += 1;   /* 保留 */
    RebarScheme* schemes[4] = {&o.bottom_x, &o.bottom_y, &o.top_x, &o.top_y};
    for (int i = 0; i < 4; ++i) {
        schemes[i]->bar_diameter_mm = p[off++];
        schemes[i]->bar_spacing_mm = p[off++] * 5.0;
        schemes[i]->area_mm2_per_m = get_u16_le(p + off) * 10.0; off += 2;
        schemes[i]->moment_capacity_kNm = get_u16_le(p + off) / 10.0; off += 2;
        schemes[i]->steel_stress_MPa = get_u16_le(p + off) * 0.1; off += 2;
        schemes[i]->crack_width_mm = get_u16_le(p + off) / 1000.0; off += 2;
    }
    o.total_rebar_ratio = get_u16_le(p + off) / 1.0e5; off += 2;
    const char* s = reinterpret_cast<const char*>(p + off);
    o.summary.assign(s, strnlen(s, kOptBlobSummarySize));
    return true;
}

EmbeddedSystemController::EmbeddedSystemController() = default;

/* ------------------------------------------------------------------------ */
/* 初始化                                             */
/* ------------------------------------------------------------------------ */
void EmbeddedSystemController::initialize() {
    hal_uart_flush();
    hal_can_flush();
    shared_mem_reset();
    hal_flash_reset();               /* 模拟出厂 Flash；目标板上为挂载扫描 */
    storage_.initialize();           /* 扫描闪存、重建哈希索引 */
    boot_timestamp_ = hal_get_tick_ms();
    calc_queue_ = TaskQueue<DesignParam, 8>();
    storage_queue_ = TaskQueue<std::pair<StoreType, std::vector<uint8_t>>, 16>();
}

/* ------------------------------------------------------------------------ */
/* 参数解析任务（优先级 3）                                                    */
/* ------------------------------------------------------------------------ */
bool EmbeddedSystemController::submit_design_frame(const uint8_t* data,
                                                   uint16_t len) {
    last_report_ = PipelineReport();
    PipelineReport& rep = last_report_;
    const uint32_t t0 = hal_get_tick_ms();

    /* 低功耗调度：计算时段升至 400MHz */
    hal_set_cpu_freq(CpuFreq::FREQ_400MHZ);

    if (!parser_.Parse_Param(data, len, &rep.param)) {
        rep.stage = DesignStage::REJECTED;
        rep.error = parser_.last_error();
        rep.elapsed_ms = hal_get_tick_ms() - t0;
        hal_set_cpu_freq(CpuFreq::FREQ_200MHZ);
        return false;
    }
    rep.ack_sent = true;
    rep.stage = DesignStage::PARSED;
    rep.elapsed_ms = hal_get_tick_ms() - t0;
    return true;
}

/* ------------------------------------------------------------------------ */
/* 结构计算任务（优先级 4，独占 FPU）                                          */
/* ------------------------------------------------------------------------ */
bool EmbeddedSystemController::run_structural_calculation() {
    PipelineReport& rep = last_report_;
    if (rep.stage != DesignStage::PARSED) return false;

    ParameterError err;
    if (!calculator_.perform_structural_analysis(rep.param, rep.calc, err)) {
        rep.error = err;
        rep.stage = DesignStage::REJECTED;
        hal_set_cpu_freq(CpuFreq::FREQ_200MHZ);
        return false;
    }
    rep.stage = DesignStage::CALCULATED;

    /* 过程数据入存储队列 */
    const uint8_t* raw = reinterpret_cast<const uint8_t*>(&rep.calc);
    std::vector<uint8_t> blob(raw, raw + sizeof(CalcResult));
    storage_queue_.send({STORE_CALC_RESULT, blob});
    return true;
}

/* ------------------------------------------------------------------------ */
/* 配筋优化（结构计算任务的后续阶段，经共享内存交付）                          */
/* ------------------------------------------------------------------------ */
static bool recalc_callback(DesignParam& param, CalcResult& calc,
                            ParameterError& error) {
    StructuralCalculator calc_machine;
    return calc_machine.perform_structural_analysis(param, calc, error);
}

bool EmbeddedSystemController::run_result_packaging() {
    PipelineReport& rep = last_report_;
    if (rep.stage != DesignStage::CALCULATED) return false;

    const uint32_t t0 = hal_get_tick_ms();
    rep.design_number = next_design_number();

    /* 配筋优化模块 */
    ParameterError err;
    if (!optimizer_.optimize(rep.param, rep.calc, rep.opt, recalc_callback, err)) {
        rep.error = err;
        hal_set_cpu_freq(CpuFreq::FREQ_200MHZ);
        return false;
    }
    rep.opt.design_number = rep.design_number;
    rep.opt.timestamp = hal_get_tick_ms();
    rep.stage = DesignStage::OPTIMIZED;

    /* 优化结果经共享内存交付（0x24000000 + 标志位 + CRC32） */
    if (!write_opt_to_shared_mem(rep.opt)) {
        rep.error = {ParameterErrorCode::UNKNOWN_ERROR, "共享内存写入失败"};
        return false;
    }
    OptResult from_mem;
    if (!read_opt_from_shared_mem(from_mem)) {
        rep.error = {ParameterErrorCode::UNKNOWN_ERROR, "共享内存 CRC 校验失败"};
        return false;
    }
    rep.opt = from_mem;   /* 以共享内存回读数据为准（校验通过的传输语义） */

    /* 结果封装模块 */
    packager_.set_check_context(
        rep.calc.deflection_check.long_term_deflection_m * 1000.0,
        rep.calc.deflection_check.allowable_deflection_m * 1000.0,
        rep.calc.crack_check.max_crack_width_mm,
        rep.calc.deflection_check.passed,
        rep.calc.crack_check.passed);

    const uint8_t status = (rep.calc.deflection_check.passed &&
                            rep.calc.crack_check.passed &&
                            rep.opt.all_checks_passed) ? 0x00 : 0x01;
    uint8_t out[kMaxOutputFrameSize];
    uint16_t out_len = 0;
    if (!packager_.Package_Result(rep.opt, rep.param, status, out, &out_len)) {
        rep.error = {ParameterErrorCode::UNKNOWN_ERROR, "结果帧封装失败"};
        return false;
    }
    rep.output_frame.assign(out, out + out_len);
    rep.stage = DesignStage::PACKAGED;
    rep.elapsed_ms += hal_get_tick_ms() - t0;
    hal_set_cpu_freq(CpuFreq::FREQ_200MHZ);   /* 回到低功耗档 */
    return true;
}

/* ------------------------------------------------------------------------ */
/* 数据存储任务（优先级 1，后台）                                              */
/* ------------------------------------------------------------------------ */
void EmbeddedSystemController::run_storage_task() {
    std::pair<StoreType, std::vector<uint8_t>> item;
    while (storage_queue_.receive(item)) {
        const uint32_t design_no = last_report_.design_number;
        /* 设计输入参数实时落盘 */
        if (item.first == STORE_CALC_RESULT) {
            const uint8_t* raw = reinterpret_cast<const uint8_t*>(&last_report_.param);
            std::vector<uint8_t> param_blob(raw, raw + sizeof(DesignParam));
            storage_.Store_Data(STORE_DESIGN_PARAM, param_blob.data(),
                                static_cast<uint16_t>(param_blob.size()), design_no);
        }
        storage_.Store_Data(item.first, item.second.data(),
                            static_cast<uint16_t>(item.second.size()), design_no);
        /* 封装后的输出帧同样存档 */
        if (!last_report_.output_frame.empty()) {
            storage_.Store_Data(STORE_PACKED_FRAME,
                                last_report_.output_frame.data(),
                                static_cast<uint16_t>(last_report_.output_frame.size()),
                                design_no);
        }
    }
}

/* ------------------------------------------------------------------------ */
/* 系统监控任务（优先级 0）                                                    */
/* ------------------------------------------------------------------------ */
SystemHealthReport EmbeddedSystemController::run_monitor_task() {
    SystemHealthReport hr;
    hr.memory_usage_ratio = mem_pool_.usage_ratio();
    hr.estimated_power_mw = hal_get_estimated_power_mw();
    hr.current_freq = hal_get_cpu_freq();
    hr.uptime_ms = hal_get_tick_ms() - boot_timestamp_;
    hr.abnormal = hr.memory_usage_ratio >= 0.8;   /* 内存监控 */
    hr.note = hr.abnormal ? "内存占用超 80%，触发存储释放" : "系统运行正常";
    return hr;
}

/* ------------------------------------------------------------------------ */
/* 共享内存读写（0x24000000 + 标志位 0xFF + CRC32）               */
/* ------------------------------------------------------------------------ */
bool EmbeddedSystemController::write_opt_to_shared_mem(const OptResult& opt) {
    std::vector<uint8_t> blob;
    serialize_opt_result(opt, blob);
    return shared_mem_write(blob.data(), blob.size());
}

bool EmbeddedSystemController::read_opt_from_shared_mem(OptResult& opt) {
    if (!shared_mem_flag_raised()) return false;
    uint8_t buf[kOptBlobSize];
    if (!shared_mem_read(buf, kOptBlobSize)) return false;
    return deserialize_opt_result(buf, kOptBlobSize, opt);
}

/* ------------------------------------------------------------------------ */
/* 完整同步流水线（等价于五任务按优先级协作一轮）                              */
/* ------------------------------------------------------------------------ */
bool EmbeddedSystemController::process_one_design(const uint8_t* data,
                                                  uint16_t len,
                                                  PipelineReport& report) {
    const bool submit_ok = submit_design_frame(data, len);
    report = last_report_;
    if (!submit_ok) return false;

    /* 计算队列（队列长度 8） */
    if (!calc_queue_.send(report.param)) {
        report.error = {ParameterErrorCode::BUFFER_TOO_SMALL, "计算队列已满"};
        return false;
    }
    if (!run_structural_calculation()) {
        report = last_report_;
        return false;
    }
    if (!run_result_packaging()) {
        report = last_report_;
        return false;
    }
    run_storage_task();
    report = last_report_;
    return true;
}

}  /* namespace rfds */
