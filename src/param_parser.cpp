/**
 * param_parser.cpp —— 模块一：参数解析模块实现
 *
 * 实现说明：
 *  - 单位制由协议帧明确约定（长度 mm、荷载 kPa），不存在动态单位；
 *  - 混凝土强度等级检查对原始等级数值做 C20~C80 区间判断；
 *  - 双向板长宽比 <= 3.0、单向板 >= 2.0（SL 191-2008）；
 *  - 增加荷载取值范围、板厚范围与 IEEE754 NaN/Inf 防护检查。
 */
#include "param_parser.h"
#include "hal.h"

#include <cmath>
#include <cstring>
#include <cstdio>
#include <algorithm>

namespace rfds {

/* ------------------------------------------------------------------------ */
/* 混凝土强度设计值数据库（对应"规范参数数据表"）                    */
/*  键：强度等级数值；值：{抗压强度设计值 fc, 抗拉强度设计值 ft} 单位 MPa      */
/* ------------------------------------------------------------------------ */
static const struct { int grade; double fc; double ft; } kConcreteTable[] = {
    {20, 11.9, 1.03}, {25, 14.3, 1.24}, {30, 16.7, 1.45}, {35, 19.1, 1.66},
    {40, 21.5, 1.87}, {45, 23.9, 2.08}, {50, 26.3, 2.29}, {55, 28.7, 2.50},
    {60, 31.1, 2.71}, {65, 33.5, 2.92}, {70, 35.9, 3.13}, {75, 38.3, 3.34},
    {80, 40.7, 3.55},
};

bool ParameterParser::query_concrete_strength(int grade, double* fc, double* ft) {
    for (const auto& row : kConcreteTable) {
        if (row.grade == grade) {
            if (fc) *fc = row.fc;
            if (ft) *ft = row.ft;
            return true;
        }
    }
    return false;   /* 查无此等级：非法输入 */
}

/* ------------------------------------------------------------------------ */
/* 第 1、2 步：帧同步 + CRC 校验 + 载荷提取（protocol.cpp 已实现）             */
/* 第 3 步：多级解码与数值映射                           */
/* ------------------------------------------------------------------------ */
bool ParameterParser::decode_and_map_parameters(const std::vector<uint8_t>& payload,
                                                uint8_t function_code,
                                                DesignParam& out, ParameterError& error) {
    out.function_code = function_code;

    if (function_code == FC_CONCRETE_SLAB_PARAM) {
        if (payload.size() < kSlabParamPayloadSize) {
            error = {ParameterErrorCode::MISSING_DATA_FIELD, "混凝土构件数据载荷不足"};
            return false;
        }
        const uint8_t* p = payload.data();

        /* 1. 混凝土强度等级（离散参数：查表映射） */
        const int grade = static_cast<int>(p[0]);
        double fc = 0.0, ft = 0.0;
        if (!query_concrete_strength(grade, &fc, &ft)) {
            error = {ParameterErrorCode::STRENGTH_GRADE_OUT_OF_RANGE,
                     "混凝土强度等级不在 C20~C80 规范序列内"};
            return false;
        }
        out.concrete_grade = grade;
        out.fc_design_MPa = fc;
        out.ft_design_MPa = ft;

        /* 2. 连续参数：IEEE754 浮点解码，并统一为国际单位制（mm -> m） */
        const float span_mm  = get_f32_le(p + 1);
        const float width_mm = get_f32_le(p + 5);
        const float thick_mm = get_f32_le(p + 9);
        const float ratio    = get_f32_le(p + 13);
        if (!std::isfinite(span_mm) || !std::isfinite(width_mm) ||
            !std::isfinite(thick_mm) || !std::isfinite(ratio)) {
            error = {ParameterErrorCode::DATA_DECODING_ERROR,
                     "浮点参数解码得到 NaN/Inf，数据非法"};
            return false;
        }
        out.slab_span_m     = static_cast<double>(span_mm) / 1000.0;
        out.slab_width_m    = static_cast<double>(width_mm) / 1000.0;
        out.slab_thickness_m = static_cast<double>(thick_mm) / 1000.0;
        out.rebar_ratio     = static_cast<double>(ratio);

        /* 3. 楼盖类型（枚举映射） */
        out.slab_type = (p[17] == 0) ? SLAB_ONE_WAY : SLAB_TWO_WAY;

        /* 4. 荷载（kPa，直接为标准单位） */
        const float dead_kPa = get_f32_le(p + 18);
        const float live_kPa = get_f32_le(p + 22);
        if (!std::isfinite(dead_kPa) || !std::isfinite(live_kPa)) {
            error = {ParameterErrorCode::DATA_DECODING_ERROR,
                     "荷载数据解码得到 NaN/Inf"};
            return false;
        }
        out.dead_load_kPa = static_cast<double>(dead_kPa);
        out.live_load_kPa = static_cast<double>(live_kPa);

        /* 5. 荷载持续时间分级 0~4 */
        out.load_duration_class = std::min(static_cast<int>(p[26]), 4);

        /* 保留位 p[27] 当前置 0，忽略 */
        return true;
    }

    if (function_code == FC_BEAM_ELEMENT_PARAM) {
        /* 肋梁参数扩展功能码（预留）：载荷 [梁长 m f32][梁宽 mm f32][梁高 mm f32] */
        if (payload.size() < 12) {
            error = {ParameterErrorCode::MISSING_DATA_FIELD, "梁构件数据载荷不足"};
            return false;
        }
        error = {ParameterErrorCode::UNSUPPORTED_FUNCTION_CODE,
                 "肋梁参数解析为预留扩展功能，当前版本不支持"};
        return false;
    }

    error = {ParameterErrorCode::UNSUPPORTED_FUNCTION_CODE, "不支持的功能码"};
    return false;
}

/* ------------------------------------------------------------------------ */
/* 第 4 步：规范阈值边界检查                             */
/* ------------------------------------------------------------------------ */
bool ParameterParser::perform_threshold_checks(const DesignParam& params,
                                               ParameterError& error) {
    using namespace kRegulation;

    /* 1. 混凝土强度等级范围（原始数值 C20~C80） */
    if (params.concrete_grade < kConcreteGradeMin ||
        params.concrete_grade > kConcreteGradeMax) {
        error = {ParameterErrorCode::STRENGTH_GRADE_OUT_OF_RANGE,
                 "混凝土强度等级超出规范范围 C20~C80"};
        return false;
    }

    /* 2. 配筋率限值 */
    if (params.rebar_ratio < kMinRebarRatio) {
        error = {ParameterErrorCode::REBAR_RATIO_OUT_OF_RANGE, "配筋率低于最小限值"};
        return false;
    }
    if (params.rebar_ratio > kMaxRebarRatio) {
        error = {ParameterErrorCode::REBAR_RATIO_OUT_OF_RANGE, "配筋率超出最大限值（超筋）"};
        return false;
    }

    /* 3. 尺寸有效性 */
    if (params.slab_span_m <= 0.0 || params.slab_width_m <= 0.0) {
        error = {ParameterErrorCode::INVALID_DATA_FORMAT,
                 "楼盖尺寸（跨度、宽度）必须为正数"};
        return false;
    }
    if (params.slab_span_m < kMinSpanM || params.slab_span_m > kMaxSpanM ||
        params.slab_width_m < kMinSpanM || params.slab_width_m > kMaxSpanM) {
        error = {ParameterErrorCode::INVALID_DATA_FORMAT,
                 "楼盖跨度/宽度超出 0.5m~12.0m 适用范围"};
        return false;
    }

    /* 4. 板厚范围 */
    if (params.slab_thickness_m < kMinSlabThicknessM ||
        params.slab_thickness_m > kMaxSlabThicknessM) {
        error = {ParameterErrorCode::THICKNESS_OUT_OF_RANGE,
                 "板厚超出规范最小/最大厚度要求"};
        return false;
    }

    /* 5. 荷载取值范围 */
    if (params.dead_load_kPa < kDeadLoadMin_kPa || params.dead_load_kPa > kDeadLoadMax_kPa ||
        params.live_load_kPa < kLiveLoadMin_kPa || params.live_load_kPa > kLiveLoadMax_kPa) {
        error = {ParameterErrorCode::LOAD_VALUE_OUT_OF_RANGE,
                 "荷载值超出规范取值范围"};
        return false;
    }

    /* 6. 长宽比与板类型逻辑一致性
     *    双向板：长宽比 <= 3.0；单向板：长宽比 >= 2.0（SL 191-2008） */
    const double aspect_ratio = params.slab_span_m / params.slab_width_m;
    if (params.slab_type == SLAB_TWO_WAY && aspect_ratio > kAspectRatioTwoWayMax) {
        error = {ParameterErrorCode::ASPECT_RATIO_INVALID,
                 "长宽比大于 3.0 应按单向板计算，与双向板声明冲突"};
        return false;
    }
    if (params.slab_type == SLAB_ONE_WAY && aspect_ratio < kAspectRatioOneWayMin) {
        error = {ParameterErrorCode::ASPECT_RATIO_INVALID,
                 "长宽比小于 2.0 应按双向板计算，与单向板声明冲突"};
        return false;
    }
    return true;
}

/* ------------------------------------------------------------------------ */
/* 完整流水线：Parse_Param                              */
/* ------------------------------------------------------------------------ */
bool ParameterParser::Parse_Param(const uint8_t* data, uint16_t len,
                                  DesignParam* design_param) {
    last_error_ = ParameterError(ParameterErrorCode::SUCCESS, "操作成功");
    if (design_param == nullptr || (data == nullptr && len > 0)) {
        last_error_ = {ParameterErrorCode::INVALID_DATA_FORMAT, "空指针入参"};
        return false;
    }

    std::vector<uint8_t> buffer(data, data + len);

    /* 1. 物理层数据接收与帧同步 */
    size_t frame_start = 0, frame_end = 0;
    if (!synchronize_frame(buffer, frame_start, frame_end)) {
        if (buffer.empty()) {
            last_error_ = {ParameterErrorCode::SUCCESS, "无数据可处理"};
        } else {
            last_error_ = {ParameterErrorCode::FRAME_LOSS,
                           "帧同步失败：未检测到完整帧头/帧尾"};
        }
        return false;
    }

    /* 2. 循环冗余校验与载荷提取 */
    RawFrame frame;
    if (!extract_and_validate_packet(buffer, frame_start, frame_end,
                                     frame, last_error_)) {
        /* 回发 NACK：携带 CRC 或协议错误码 */
        std::vector<uint8_t> nack = build_nack_frame(
            frame.device_address, frame.function_code,
            static_cast<uint8_t>(last_error_.code));
        hal_uart_send(nack.data(), nack.size());
        return false;
    }

    /* 帧内地址字段为源设备地址（外部设备地址范围 0x02~0xFF，
     * 系统 CAN 地址 0x01） */
    if (frame.device_address < kDeviceAddrMin ||
        frame.device_address > kDeviceAddrMax) {
        last_error_ = {ParameterErrorCode::INVALID_DEVICE_ADDRESS,
                       "源设备地址不在 0x02~0xFF 外部设备范围内"};
        std::vector<uint8_t> nack = build_nack_frame(
            frame.device_address, frame.function_code,
            static_cast<uint8_t>(last_error_.code));
        hal_uart_send(nack.data(), nack.size());
        return false;
    }

    /* 3. 多级解码与数值映射 */
    if (!decode_and_map_parameters(frame.payload, frame.function_code,
                                   *design_param, last_error_)) {
        std::vector<uint8_t> nack = build_nack_frame(
            frame.device_address, frame.function_code,
            static_cast<uint8_t>(last_error_.code));
        hal_uart_send(nack.data(), nack.size());
        return false;
    }
    design_param->device_address = frame.device_address;

    /* 4. 规范阈值边界检查 */
    if (!perform_threshold_checks(*design_param, last_error_)) {
        std::vector<uint8_t> nack = build_nack_frame(
            frame.device_address, frame.function_code,
            static_cast<uint8_t>(last_error_.code));
        hal_uart_send(nack.data(), nack.size());
        return false;
    }

    /* 全部通过：回 ACK */
    std::vector<uint8_t> ack = build_ack_frame(frame.device_address, frame.function_code);
    hal_uart_send(ack.data(), ack.size());
    return true;
}

}  /* namespace rfds */
