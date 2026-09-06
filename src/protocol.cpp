/**
 * protocol.cpp —— CRC 校验与帧协议实现
 */
#include "protocol.h"

#include <cstring>

namespace rfds {

/* ------------------------------------------------------------------------ */
/* CRC16-XMODEM（多项式 0x1021，初值 0xFFFF）            */
/* 参考自检向量："123456789" -> 0x29B1                                        */
/* 查表在首次调用时按定义式生成，避免手写表项笔误                              */
/* ------------------------------------------------------------------------ */
static uint16_t kCrc16Table[256];
static bool g_crc16_table_ready = false;

static void init_crc16_table() {
    for (uint32_t i = 0; i < 256; ++i) {
        uint16_t c = static_cast<uint16_t>(i << 8);
        for (int k = 0; k < 8; ++k) {
            c = (c & 0x8000u)
                ? static_cast<uint16_t>((c << 1) ^ 0x1021u)
                : static_cast<uint16_t>(c << 1);
        }
        kCrc16Table[i] = c;
    }
    g_crc16_table_ready = true;
}

uint16_t crc16(const uint8_t* data, size_t length) {
    if (!g_crc16_table_ready) init_crc16_table();
    uint16_t crc = 0xFFFF;  /* 初值 0xFFFF */
    for (size_t i = 0; i < length; ++i) {
        crc = static_cast<uint16_t>((crc << 8) ^ kCrc16Table[((crc >> 8) ^ data[i]) & 0xFF]);
    }
    return crc;
}

uint16_t crc16(const std::vector<uint8_t>& data) {
    return crc16(data.data(), data.size());
}

/* ------------------------------------------------------------------------ */
/* CRC32-IEEE 802.3：共享内存数据完整性校验                                    */
/* 参考自检向量："123456789" -> 0xCBF43926                                    */
/* ------------------------------------------------------------------------ */
static uint32_t crc32_table_initialized = 0;
static uint32_t kCrc32Table[256];

static void init_crc32_table() {
    for (uint32_t i = 0; i < 256; ++i) {
        uint32_t c = i;
        for (int k = 0; k < 8; ++k) {
            c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        }
        kCrc32Table[i] = c;
    }
    crc32_table_initialized = 1;
}

uint32_t crc32(const uint8_t* data, size_t length) {
    if (!crc32_table_initialized) init_crc32_table();
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < length; ++i) {
        crc = kCrc32Table[(crc ^ data[i]) & 0xFFu] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFFu;
}

uint32_t crc32(const std::vector<uint8_t>& data) {
    return crc32(data.data(), data.size());
}

/* ------------------------------------------------------------------------ */
/* 字节序辅助                                                                 */
/* ------------------------------------------------------------------------ */
void put_u16_le(std::vector<uint8_t>& buf, uint16_t v) {
    buf.push_back(static_cast<uint8_t>(v & 0xFF));
    buf.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
}

void put_u32_le(std::vector<uint8_t>& buf, uint32_t v) {
    buf.push_back(static_cast<uint8_t>(v & 0xFF));
    buf.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
    buf.push_back(static_cast<uint8_t>((v >> 16) & 0xFF));
    buf.push_back(static_cast<uint8_t>((v >> 24) & 0xFF));
}

void put_f32_le(std::vector<uint8_t>& buf, float v) {
    uint32_t bits;
    std::memcpy(&bits, &v, sizeof(bits));
    put_u32_le(buf, bits);
}

uint16_t get_u16_le(const uint8_t* p) {
    return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

uint32_t get_u32_le(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

float get_f32_le(const uint8_t* p) {
    uint32_t bits = get_u32_le(p);
    float v;
    std::memcpy(&v, &bits, sizeof(v));
    return v;
}

/* ------------------------------------------------------------------------ */
/* 帧同步                                                */
/* ------------------------------------------------------------------------ */
bool synchronize_frame(const std::vector<uint8_t>& buffer,
                       size_t& frame_start_idx, size_t& frame_end_idx) {
    frame_start_idx = static_cast<size_t>(-1);
    frame_end_idx = static_cast<size_t>(-1);
    const size_t buffer_size = buffer.size();
    if (buffer_size < kMinPacketSize) return false;

    for (size_t i = 0; i + kFrameHeaderSize <= buffer_size; ++i) {
        if (std::memcmp(buffer.data() + i, kFrameHeader, kFrameHeaderSize) == 0) {
            /* 自帧头之后搜索帧尾 */
            for (size_t j = i + kFrameHeaderSize;
                 j + kFrameTailSize <= buffer_size; ++j) {
                if (std::memcmp(buffer.data() + j, kFrameTail, kFrameTailSize) == 0) {
                    frame_start_idx = i;
                    frame_end_idx = j + kFrameTailSize;
                    return true;
                }
            }
            break;  /* 找到帧头但无帧尾：帧不完整 */
        }
    }
    return false;
}

/* ------------------------------------------------------------------------ */
/* 包提取与校验                                          */
/* ------------------------------------------------------------------------ */
bool extract_and_validate_packet(const std::vector<uint8_t>& buffer,
                                 size_t frame_start_idx, size_t frame_end_idx,
                                 RawFrame& frame, ParameterError& error) {
    const size_t packet_size = frame_end_idx - frame_start_idx;
    if (packet_size < kMinPacketSize) {
        error = {ParameterErrorCode::INVALID_PROTOCOL_FORMAT, "数据包长度不足"};
        return false;
    }

    /* 帧内布局: [AA 55][ADDR FC LEN_L LEN_H][DATA...][CRC_L CRC_H][55 AA] */
    const uint8_t* base = buffer.data() + frame_start_idx;
    const size_t inner_size = packet_size - kFrameHeaderSize - kFrameTailSize;
    const size_t crc_pos = kFrameHeaderSize + inner_size - kCrc16Size;
    /* inner = ADDR(1)+FC(1)+LEN(2)+DATA(n)+CRC(2)，最小 4+0+2=6 */
    if (inner_size < kPacketHeaderSize + kCrc16Size) {
        error = {ParameterErrorCode::INVALID_PROTOCOL_FORMAT, "载荷头部不完整"};
        return false;
    }

    uint16_t received_crc = get_u16_le(base + crc_pos);
    const size_t covered_size = inner_size - kCrc16Size;  /* ADDR..DATA 末字节 */
    uint16_t calculated_crc = crc16(base + kFrameHeaderSize, covered_size);
    if (calculated_crc != received_crc) {
        error = {ParameterErrorCode::CRC_ERROR, "CRC16 校验失败，数据传输畸变"};
        return false;
    }

    /* CRC 通过，提取协议头 */
    frame.device_address = base[kFrameHeaderSize];
    frame.function_code  = base[kFrameHeaderSize + 1];
    frame.data_length    = get_u16_le(base + kFrameHeaderSize + 2);

    const size_t actual_data_size = covered_size - kPacketHeaderSize;
    if (frame.data_length != actual_data_size) {
        error = {ParameterErrorCode::INVALID_PROTOCOL_FORMAT,
                 "数据长度字段与实际载荷大小不匹配"};
        return false;
    }

    frame.payload.assign(base + kFrameHeaderSize + kPacketHeaderSize,
                         base + kFrameHeaderSize + kPacketHeaderSize + actual_data_size);
    return true;
}

/* ------------------------------------------------------------------------ */
/* 组帧（模块 4 输出使用）                                                     */
/* ------------------------------------------------------------------------ */
std::vector<uint8_t> build_frame(uint8_t device_address, uint8_t function_code,
                                 const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> frame;
    frame.reserve(kMinPacketSize + payload.size());
    frame.push_back(kFrameHeader[0]);
    frame.push_back(kFrameHeader[1]);
    frame.push_back(device_address);
    frame.push_back(function_code);
    put_u16_le(frame, static_cast<uint16_t>(payload.size()));
    frame.insert(frame.end(), payload.begin(), payload.end());
    /* CRC16 覆盖 ADDR..DATA */
    uint16_t crc = crc16(frame.data() + kFrameHeaderSize,
                         frame.size() - kFrameHeaderSize);
    put_u16_le(frame, crc);
    frame.push_back(kFrameTail[0]);
    frame.push_back(kFrameTail[1]);
    return frame;
}

std::vector<uint8_t> build_ack_frame(uint8_t device_address, uint8_t function_code) {
    /* ACK 载荷：被应答设备地址 + 原功能码 */
    return build_frame(kDeviceAddressSelf, FC_ACK_RESPONSE,
                       {device_address, function_code});
}

std::vector<uint8_t> build_nack_frame(uint8_t device_address, uint8_t function_code,
                                      uint8_t error_code) {
    /* NACK 载荷：被应答设备地址 + 原功能码 + 错误码 */
    return build_frame(kDeviceAddressSelf, FC_NACK_RESPONSE,
                       {device_address, function_code, error_code});
}

}  /* namespace rfds */
