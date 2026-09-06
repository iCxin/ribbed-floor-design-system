/**
 * ============================================================================
 *  水工钢筋混凝土肋形楼盖辅助设计系统 V1.0
 *  hal.h —— 硬件抽象层（HAL）
 * ============================================================================
 *  2.2 节"硬件层 - 驱动层"描述：
 *    目标硬件为 STM32H743ZI2 + FreeRTOS。本工程在 PC 端以"驱动模拟"方式
 *    复现嵌入式驱动层接口，接口签名与 STM32 HAL 保持一一对应：
 *
 *      HAL_GetTick()            -> xTaskGetTickCount 毫秒节拍
 *      hal_uart_send/receive    -> UART RS232 9600bps 8N1
 *      hal_can_send/receive     -> CAN 2.0B 500kbps
 *      hal_set_cpu_freq         -> 动态主频调度 200/400MHz
 *      SharedMemory             -> 0x24000000 共享内存 + 标志位
 *
 *  移植说明：将模拟实现替换为 STM32 寄存器/DMA 操作即可运行于
 *  目标板，上层模块代码无需改动。
 * ============================================================================
 */
#ifndef RIBBED_FLOOR_HAL_H_
#define RIBBED_FLOOR_HAL_H_

#include <cstdint>
#include <cstddef>
#include <deque>
#include <vector>

#include "common_types.h"

namespace rfds {

/* ------------------------------------------------------------------------ */
/* 模拟系统节拍与动态主频                                                      */
/* ------------------------------------------------------------------------ */
uint32_t hal_get_tick_ms();                 /* 毫秒系统节拍（单调递增） */
void     hal_advance_tick_ms(uint32_t ms);  /* 模拟时间推进（测试用）   */

/* CPU 主频档位 */
enum class CpuFreq { FREQ_200MHZ = 200, FREQ_400MHZ = 400 };
void     hal_set_cpu_freq(CpuFreq freq);
CpuFreq  hal_get_cpu_freq();
double   hal_get_estimated_power_mw();      /* 功耗估算（规范限值 <=1.2W） */

/* ------------------------------------------------------------------------ */
/* UART 模拟（RS232 9600bps 8N1，硬件接口）                         */
/*  外部设备通过 hal_uart_inject() 注入原始字节流，                           */
/*  系统 NACK/ACK/结果帧经 hal_uart_send() 进入发送队列供外部读取。           */
/* ------------------------------------------------------------------------ */
void hal_uart_inject(const uint8_t* data, size_t len);   /* 模拟外部输入 */
void hal_uart_inject(const std::vector<uint8_t>& data);
bool hal_uart_receive(uint8_t* buf, size_t len, size_t* received);
bool hal_uart_send(const uint8_t* data, size_t len);     /* 结果帧输出   */
size_t hal_uart_tx_available();
bool hal_uart_tx_read(std::vector<uint8_t>& out);
void hal_uart_flush();                                   /* 清空收发缓冲 */

/* ------------------------------------------------------------------------ */
/* CAN 模拟（500kbps，CAN 2.0B，）                                 */
/* ------------------------------------------------------------------------ */
struct CanFrame {
    uint32_t id;                 /* 11 位标准 ID；系统地址 0x01 */
    std::vector<uint8_t> data;   /* 最长 8 字节数据域           */
};
void hal_can_inject(const CanFrame& frame);
bool hal_can_receive(CanFrame& frame);
bool hal_can_send(const CanFrame& frame);
void hal_can_flush();

/* ------------------------------------------------------------------------ */
/* 共享内存模拟（配筋优化模块 <-> 结构计算模块）                  */
/*  目标板映射到 0x24000000 起始的 AXI SRAM；PC 模拟使用静态缓冲区。          */
/*  布局（总 256B，标志位固定 @ 0xFF）：                   */
/*    DATA  : 0x00 ~ 0xFA （有效数据区，最大 251B）                           */
/*    CRC32 : 0xFB ~ 0xFE （数据完整性校验，）                     */
/*    FLAG  : 0xFF       （写 1 通知读取方，读取校验通过后清零）              */
/* ------------------------------------------------------------------------ */
constexpr uintptr_t kSharedMemAddr = 0x24000000;   /* 目标板物理地址 */
constexpr size_t    kSharedMemDataSize = 256;
constexpr size_t    kSharedMemCrcOffset = 0xFB;
constexpr size_t    kSharedMemFlagOffset = 0xFF;
constexpr size_t    kSharedMemTotalSize = kSharedMemFlagOffset + 1;  /* 256B */
constexpr size_t    kSharedMemUsableSize = kSharedMemCrcOffset;      /* 251B */

/* 写入共享内存：自动计算并附加 CRC32，写入后置标志位 */
bool shared_mem_write(const uint8_t* data, size_t len);
/* 读取共享内存：校验 CRC32 与标志位，成功时清除标志位（单次读取语义） */
bool shared_mem_read(uint8_t* out, size_t len);
bool shared_mem_flag_raised();
void shared_mem_reset();

/* ------------------------------------------------------------------------ */
/* Flash 存储模拟（NOR Flash 本地数据库）                        */
/*  页写入 / 扇区擦除 / 磨损计数，供数据存储模块使用。                        */
/* ------------------------------------------------------------------------ */
constexpr size_t kFlashPageSize = 256;         /* NOR Flash 页大小 (B) */
constexpr size_t kFlashSectorSize = 4096;      /* 扇区大小 4KB         */
constexpr size_t kFlashSectorCount = 32;       /* 模拟 128KB NOR Flash */

enum HalFlashStatus { HAL_FLASH_OK = 0, HAL_FLASH_ERR_ERASE_BEFORE_WRITE = -1,
                      HAL_FLASH_ERR_RANGE = -2 };

HalFlashStatus hal_flash_read(size_t addr, uint8_t* buf, size_t len);
HalFlashStatus hal_flash_write_page(size_t addr, const uint8_t* buf, size_t len);
HalFlashStatus hal_flash_erase_sector(size_t sector_index);
uint32_t       hal_flash_get_erase_count(size_t sector_index);
void           hal_flash_reset();              /* 恢复出厂（测试用） */

}  /* namespace rfds */

#endif  /* RIBBED_FLOOR_HAL_H_ */
