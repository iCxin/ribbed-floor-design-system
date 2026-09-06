/**
 * result_pack.cpp —— 模块四：结果封装模块实现
 *
 * 实现说明：钢筋直径采用 4bit 编码表（含"无钢筋"=0），定点数输出
 * 均做饱和处理，防止异常数据截断回绕。
 */
#include "result_pack.h"
#include "hal.h"

#include <cstring>
#include <algorithm>

namespace rfds {

/* ------------------------------------------------------------------------ */
/* 钢筋直径 4 比特编码                                                         */
/* ------------------------------------------------------------------------ */
int encode_bar_diameter(int diameter_mm) {
    static const int table[] = {8, 10, 12, 14, 16, 18, 20, 22, 25};
    for (int i = 0; i < 9; ++i)
        if (table[i] == diameter_mm) return i + 1;
    return 0;
}

int decode_bar_diameter(int code) {
    static const int table[] = {8, 10, 12, 14, 16, 18, 20, 22, 25};
    if (code < 1 || code > 9) return 0;
    return table[code - 1];
}

/* 定点数饱和转换 */
static uint16_t saturate_u16(double v) {
    if (v <= 0.0) return 0;
    if (v >= 65535.0) return 65535;
    return static_cast<uint16_t>(v + 0.5);
}

static uint8_t pack_bar_byte(const RebarScheme& s) {
    const int dia_code = encode_bar_diameter(s.bar_diameter_mm);
    const int spacing_code =
        std::min(15, static_cast<int>(s.bar_spacing_mm / 10.0 + 0.5));
    return static_cast<uint8_t>((dia_code << 4) | (dia_code ? spacing_code : 0));
}

/* ------------------------------------------------------------------------ */
/* 载荷构建                       */
/* ------------------------------------------------------------------------ */
bool ResultPackager::build_result_payload(const OptResult& opt_result,
                                          const DesignParam& param,
                                          uint8_t status_code,
                                          std::vector<uint8_t>& payload) {
    payload.clear();
    payload.reserve(kResultPayloadSize);

    std::vector<uint8_t> body;   /* 除尾部 CRC32 外的载荷体 */
    body.push_back(status_code);                                   /* 偏移 0  */
    put_u32_le(body, opt_result.design_number);                    /* 1~4    */
    put_u32_le(body, opt_result.timestamp);                        /* 5~8    */
    put_u16_le(body, saturate_u16(param.slab_span_m * 1000.0));    /* 9~10   */
    put_u16_le(body, saturate_u16(param.slab_width_m * 1000.0));   /* 11~12  */

    /* 挠度定点数：保留两位小数 ×100 */
    put_u16_le(body, saturate_u16(defl_mm_ * 100.0));               /* 13~14 */
    put_u16_le(body, saturate_u16(allow_defl_mm_ * 100.0));         /* 15~16 */
    put_u16_le(body, saturate_u16(crack_mm_ * 1000.0));             /* 17~18 */
    put_u16_le(body, saturate_u16(opt_result.steel_weight_kg * 10.0)); /* 19~20 */

    uint8_t flags = 0;
    if (defl_pass_) flags |= 0x01;
    if (crack_pass_) flags |= 0x02;
    if (opt_result.safety_factor >= 1.0) flags |= 0x04;
    body.push_back(flags);                                         /* 21    */

    body.push_back(pack_bar_byte(opt_result.bottom_x));            /* 22    */
    body.push_back(pack_bar_byte(opt_result.bottom_y));            /* 23    */
    body.push_back(pack_bar_byte(opt_result.top_x));               /* 24    */
    body.push_back(pack_bar_byte(opt_result.top_y));               /* 25    */

    put_u16_le(body, saturate_u16(opt_result.total_rebar_ratio * 1.0e5));  /* 26~27 */
    put_u16_le(body, saturate_u16(opt_result.safety_factor * 100.0));      /* 28~29 */

    body.push_back(static_cast<uint8_t>(
        std::min(255, opt_result.iterations_used)));               /* 30    */
    body.push_back(opt_result.constructive_adjusted ? 0x01 : 0x00);/* 31    */
    body.push_back(static_cast<uint8_t>(param.concrete_grade));    /* 32    */
    body.push_back(static_cast<uint8_t>(param.slab_type));         /* 33    */
    body.push_back(0x01);   /* 载荷版本 */                          /* 34    */
    body.push_back(0x00);   /* 保留 */                              /* 35    */

    /* CRC32 校验（共享内存/序列化数据完整性校验） */
    const uint32_t crc = crc32(body);
    put_u32_le(body, crc);                                         /* 36~39 */

    payload = body;
    return payload.size() == kResultPayloadSize;
}

/* ------------------------------------------------------------------------ */
/* Package_Result                                           */
/* ------------------------------------------------------------------------ */
bool ResultPackager::Package_Result(const OptResult& opt_result,
                                    const DesignParam& param,
                                    uint8_t status_code,
                                    uint8_t* output, uint16_t* out_len) {
    if (output == nullptr || out_len == nullptr) return false;
    std::vector<uint8_t> payload;
    if (!build_result_payload(opt_result, param, status_code, payload)) return false;

    std::vector<uint8_t> frame = build_frame(kDeviceAddressSelf,
                                             FC_RESULT_PACKAGE, payload);
    if (frame.size() > kMaxOutputFrameSize) return false;

    std::memcpy(output, frame.data(), frame.size());
    *out_len = static_cast<uint16_t>(frame.size());

    /* 双缓冲交替发送 */
    return send_frame_double_buffered(frame);
}

/* ------------------------------------------------------------------------ */
/* 双缓冲交替发送                                        */
/*  缓冲区 A 填充时，DMA 可并行发送缓冲区 B 中上一帧；PC 模拟以两个缓冲       */
/*  轮转 + 即时 UART 输出复现零等待语义。                                     */
/* ------------------------------------------------------------------------ */
bool ResultPackager::send_frame_double_buffered(const std::vector<uint8_t>& frame) {
    constexpr int kBufferCount = 2;
    static std::vector<uint8_t> buffers[kBufferCount];
    static int active = 0;

    /* 切换到备用缓冲（模拟 DMA 正在发送另一缓冲） */
    active = (active + 1) % kBufferCount;
    buffers[active] = frame;

    /* 经 UART 输出（模拟 DMA 传输完成中断触发） */
    return hal_uart_send(buffers[active].data(), buffers[active].size());
}

}  /* namespace rfds */
