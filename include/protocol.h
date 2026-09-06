/**
 * ============================================================================
 *  水工钢筋混凝土肋形楼盖辅助设计系统 V1.0
 *  protocol.h —— 通信协议与校验算法
 * ============================================================================
 *  对应4.2 节软硬件接口设计、6.1/6.4 节帧同步与 CRC 描述：
 *
 *  帧格式（变长帧，帧头 0xAA 0x55，帧尾 0x55 0xAA）：
 *    +--------+--------+--------+---------+--------+--------+---------+
 *    | 0xAA   | 0x55   | ADDR   | FC      | LEN(2) | DATA.. | CRC16(2)| 0x55 0xAA
 *    +--------+--------+--------+---------+--------+--------+---------+
 *      帧头      帧头    设备地址  功能码   数据长度  参数载荷   校验码    帧尾
 *
 *  - CRC16 覆盖范围：ADDR 起至 DATA 末字节（不含帧头、CRC、帧尾）
 *  - CRC16 算法：XMODEM 多项式 0x1021，初始值 0xFFFF
 *  - CRC32 算法：IEEE 802.3，用于共享内存数据完整性校验
 * ============================================================================
 */
#ifndef RIBBED_FLOOR_PROTOCOL_H_
#define RIBBED_FLOOR_PROTOCOL_H_

#include <cstdint>
#include <cstddef>
#include <vector>

#include "common_types.h"

namespace rfds {

/* 帧结构常量 */
constexpr uint8_t kFrameHeader[2] = {0xAA, 0x55};
constexpr uint8_t kFrameTail[2]   = {0x55, 0xAA};
constexpr size_t  kFrameHeaderSize = 2;
constexpr size_t  kFrameTailSize = 2;
constexpr size_t  kCrc16Size = 2;
constexpr size_t  kPacketHeaderSize = 4;  /* ADDR(1)+FC(1)+LEN(2) */
constexpr size_t  kMinPacketSize = kFrameHeaderSize + kPacketHeaderSize + kCrc16Size + kFrameTailSize;
constexpr uint32_t kFrameTimeoutMs = 100; /* 帧超时时间 */

/* 参数载荷 FC 0x01 布局（完善：在字节基础上扩展荷载与板厚，共 28 字节）
 * 偏移  类型      字段
 *  0    u8        混凝土强度等级数值（20/25/.../80）
 *  1    f32 LE    楼盖跨度 X（mm）
 *  5    f32 LE    楼盖宽度 Y（mm）
 *  9    f32 LE    板厚（mm）
 * 13    f32 LE    配筋率（无量纲）
 * 17    u8        楼盖类型（0 单向 / 1 双向）
 * 18    f32 LE    恒荷载（kPa）
 * 22    f32 LE    活荷载（kPa）
 * 26    u8        荷载持续时间分级 0~4
 * 27    u8        保留（置 0）                                                    */
constexpr size_t kSlabParamPayloadSize = 28;

/* 计算结果输出帧（模块 4，约束输出长度 <= 128 字节） */
constexpr size_t kMaxOutputFrameSize = 128;

/* ------------------------------------------------------------------------ */
/* CRC 校验                                                                   */
/* ------------------------------------------------------------------------ */
/* CRC16：XMODEM 多项式 0x1021，初始值 0xFFFF */
uint16_t crc16(const uint8_t* data, size_t length);
uint16_t crc16(const std::vector<uint8_t>& data);

/* CRC32：IEEE 802.3（poly 0xEDB88320 反射），初值 0xFFFFFFFF，终值取反
 * 用于共享内存写入数据的完整性校验 */
uint32_t crc32(const uint8_t* data, size_t length);
uint32_t crc32(const std::vector<uint8_t>& data);

/* ------------------------------------------------------------------------ */
/* 字节序辅助（协议统一小端 LE）                                              */
/* ------------------------------------------------------------------------ */
void     put_u16_le(std::vector<uint8_t>& buf, uint16_t v);
void     put_u32_le(std::vector<uint8_t>& buf, uint32_t v);
void     put_f32_le(std::vector<uint8_t>& buf, float v);
uint16_t get_u16_le(const uint8_t* p);
uint32_t get_u32_le(const uint8_t* p);
float    get_f32_le(const uint8_t* p);

/* ------------------------------------------------------------------------ */
/* 帧同步与包提取（模块 1 使用；模块 4 组帧后经此反向验证）                    */
/* ------------------------------------------------------------------------ */
struct RawFrame {
    uint8_t  device_address;
    uint8_t  function_code;
    uint16_t data_length;
    std::vector<uint8_t> payload;   /* 剥离 ADDR/FC/LEN 后的纯净参数载荷 */
};

/* 帧同步：扫描缓冲区寻找 [帧头 ... 帧尾] 完整帧，返回起止下标（含帧尾） */
bool synchronize_frame(const std::vector<uint8_t>& buffer,
                       size_t& frame_start_idx, size_t& frame_end_idx);

/* 包提取与校验：CRC16 比对、长度一致性校验、剥离帧头帧尾与协议头 */
bool extract_and_validate_packet(const std::vector<uint8_t>& buffer,
                                 size_t frame_start_idx, size_t frame_end_idx,
                                 RawFrame& frame, ParameterError& error);

/* 组帧：将 ADDR/FC/载荷 组装为带 CRC16 的完整帧（模块 4 输出驱动使用） */
std::vector<uint8_t> build_frame(uint8_t device_address, uint8_t function_code,
                                 const std::vector<uint8_t>& payload);

/* 应答帧（ACK/NACK，） */
std::vector<uint8_t> build_ack_frame(uint8_t device_address, uint8_t function_code);
std::vector<uint8_t> build_nack_frame(uint8_t device_address, uint8_t function_code,
                                      uint8_t error_code);

}  /* namespace rfds */

#endif  /* RIBBED_FLOOR_PROTOCOL_H_ */
