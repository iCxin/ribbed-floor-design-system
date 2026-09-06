/**
 * hal_mock.cpp —— 硬件抽象层 PC 模拟实现
 *
 * 每一个函数对应 STM32 HAL / CMSIS 驱动的同名职责，移植时仅需将本文件
 * 替换为基于 STM32H743 寄存器的实现，上层五个模块代码零修改。
 */
#include "hal.h"
#include "protocol.h"

#include <cstdio>
#include <cstring>
#include <algorithm>

namespace rfds {

/* ------------------------------------------------------------------------ */
/* 系统节拍（对应 FreeRTOS xTaskGetTickCount）                                 */
/* ------------------------------------------------------------------------ */
static uint32_t g_tick_ms = 0;

uint32_t hal_get_tick_ms() { return g_tick_ms; }
void hal_advance_tick_ms(uint32_t ms) { g_tick_ms += ms; }

/* ------------------------------------------------------------------------ */
/* 动态主频与功耗                                   */
/* ------------------------------------------------------------------------ */
static CpuFreq g_cpu_freq = CpuFreq::FREQ_200MHZ;

void hal_set_cpu_freq(CpuFreq freq) { g_cpu_freq = freq; }
CpuFreq hal_get_cpu_freq() { return g_cpu_freq; }

double hal_get_estimated_power_mw() {
    /* ：正常运行功耗 <=1.2W，低功耗 <=0.3W */
    switch (g_cpu_freq) {
        case CpuFreq::FREQ_400MHZ: return 1150.0;  /* 1.15W < 1.2W */
        case CpuFreq::FREQ_200MHZ: return 280.0;   /* 0.28W < 0.3W */
    }
    return 0.0;
}

/* ------------------------------------------------------------------------ */
/* UART 模拟                                                                  */
/* ------------------------------------------------------------------------ */
static std::deque<uint8_t> g_uart_rx;   /* 外部设备 -> 系统 */
static std::deque<uint8_t> g_uart_tx;   /* 系统 -> 外部设备 */

void hal_uart_inject(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; ++i) g_uart_rx.push_back(data[i]);
}
void hal_uart_inject(const std::vector<uint8_t>& data) {
    hal_uart_inject(data.data(), data.size());
}
bool hal_uart_receive(uint8_t* buf, size_t len, size_t* received) {
    size_t n = std::min(len, g_uart_rx.size());
    for (size_t i = 0; i < n; ++i) { buf[i] = g_uart_rx.front(); g_uart_rx.pop_front(); }
    if (received) *received = n;
    return n > 0;
}
bool hal_uart_send(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; ++i) g_uart_tx.push_back(data[i]);
    return true;
}
size_t hal_uart_tx_available() { return g_uart_tx.size(); }
bool hal_uart_tx_read(std::vector<uint8_t>& out) {
    out.assign(g_uart_tx.begin(), g_uart_tx.end());
    g_uart_tx.clear();
    return !out.empty();
}
void hal_uart_flush() { g_uart_rx.clear(); g_uart_tx.clear(); }

/* ------------------------------------------------------------------------ */
/* CAN 模拟                                                                   */
/* ------------------------------------------------------------------------ */
static std::deque<CanFrame> g_can_rx;
static std::deque<CanFrame> g_can_tx;

void hal_can_inject(const CanFrame& frame) { g_can_rx.push_back(frame); }
bool hal_can_receive(CanFrame& frame) {
    if (g_can_rx.empty()) return false;
    frame = g_can_rx.front(); g_can_rx.pop_front();
    return true;
}
bool hal_can_send(const CanFrame& frame) { g_can_tx.push_back(frame); return true; }
void hal_can_flush() { g_can_rx.clear(); g_can_tx.clear(); }

/* ------------------------------------------------------------------------ */
/* 共享内存模拟（布局与目标板 0x24000000 一致）                                */
/* ------------------------------------------------------------------------ */
static uint8_t g_shared_mem[kSharedMemTotalSize];

bool shared_mem_write(const uint8_t* data, size_t len) {
    if (len > kSharedMemUsableSize) return false;
    std::memset(g_shared_mem, 0, kSharedMemTotalSize);
    if (len > 0 && data) std::memcpy(g_shared_mem, data, len);
    /* CRC32 写入 0xFB~0xFE */
    uint32_t crc = crc32(data, len);
    uint8_t crc_bytes[4] = {
        static_cast<uint8_t>(crc & 0xFF), static_cast<uint8_t>((crc >> 8) & 0xFF),
        static_cast<uint8_t>((crc >> 16) & 0xFF), static_cast<uint8_t>((crc >> 24) & 0xFF)};
    std::memcpy(g_shared_mem + kSharedMemCrcOffset, crc_bytes, 4);
    /* 标志位置 1 通知读取方，读取校验通过后清零 */
    g_shared_mem[kSharedMemFlagOffset] = 1;
    return true;
}

bool shared_mem_flag_raised() { return g_shared_mem[kSharedMemFlagOffset] == 1; }

bool shared_mem_read(uint8_t* out, size_t len) {
    if (len > kSharedMemUsableSize) return false;
    if (g_shared_mem[kSharedMemFlagOffset] != 1) return false;
    uint32_t crc_stored =
        static_cast<uint32_t>(g_shared_mem[kSharedMemCrcOffset]) |
        (static_cast<uint32_t>(g_shared_mem[kSharedMemCrcOffset + 1]) << 8) |
        (static_cast<uint32_t>(g_shared_mem[kSharedMemCrcOffset + 2]) << 16) |
        (static_cast<uint32_t>(g_shared_mem[kSharedMemCrcOffset + 3]) << 24);
    if (crc_stored != crc32(g_shared_mem, len)) return false;   /* 数据被破坏 */
    if (len > 0 && out) std::memcpy(out, g_shared_mem, len);
    g_shared_mem[kSharedMemFlagOffset] = 0;   /* 单次读取语义 */
    return true;
}

void shared_mem_reset() { std::memset(g_shared_mem, 0, kSharedMemTotalSize); }

/* ------------------------------------------------------------------------ */
/* NOR Flash 模拟                                                             */
/* ------------------------------------------------------------------------ */
static uint8_t  g_flash[kFlashSectorCount * kFlashSectorSize];
static uint32_t g_erase_count[kFlashSectorCount];
static bool     g_flash_sectors_erased[kFlashSectorCount];

HalFlashStatus hal_flash_read(size_t addr, uint8_t* buf, size_t len) {
    if (addr + len > sizeof(g_flash)) return HAL_FLASH_ERR_RANGE;
    std::memcpy(buf, g_flash + addr, len);
    return HAL_FLASH_OK;
}

HalFlashStatus hal_flash_write_page(size_t addr, const uint8_t* buf, size_t len) {
    if (addr + len > sizeof(g_flash)) return HAL_FLASH_ERR_RANGE;
    /* NOR Flash 特性：编程只能将位 1 -> 0，写入前必须擦除
     * （对应"先擦后写"的原子写入约束） */
    for (size_t i = 0; i < len; ++i) {
        if ((g_flash[addr + i] & buf[i]) != buf[i])
            return HAL_FLASH_ERR_ERASE_BEFORE_WRITE;
    }
    std::memcpy(g_flash + addr, buf, len);
    return HAL_FLASH_OK;
}

HalFlashStatus hal_flash_erase_sector(size_t sector_index) {
    if (sector_index >= kFlashSectorCount) return HAL_FLASH_ERR_RANGE;
    std::memset(g_flash + sector_index * kFlashSectorSize, 0xFF, kFlashSectorSize);
    ++g_erase_count[sector_index];
    g_flash_sectors_erased[sector_index] = true;
    return HAL_FLASH_OK;
}

uint32_t hal_flash_get_erase_count(size_t sector_index) {
    if (sector_index >= kFlashSectorCount) return 0;
    return g_erase_count[sector_index];
}

void hal_flash_reset() {
    std::memset(g_flash, 0xFF, sizeof(g_flash));
    std::memset(g_erase_count, 0, sizeof(g_erase_count));
    std::fill(g_flash_sectors_erased, g_flash_sectors_erased + kFlashSectorCount, true);
}

}  /* namespace rfds */
