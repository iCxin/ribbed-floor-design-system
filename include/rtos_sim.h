/**
 * ============================================================================
 *  水工钢筋混凝土肋形楼盖辅助设计系统 V1.0
 *  rtos_sim.h —— FreeRTOS 任务调度模拟层
 * ============================================================================
 *  2.2 节操作系统层、5.2 节任务调度设计：
 *
 *    结构计算任务  优先级 4（最高，独占 FPU）
 *    参数解析任务  优先级 3
 *    结果封装任务  优先级 2
 *    数据存储任务  优先级 1
 *    系统监控任务  优先级 0
 *
 *  目标板上为 5 个独立 FreeRTOS Task + xQueueSend/xQueueReceive；
 *  PC 模拟层以"协作式任务队列 + 优先级调度"复现同等语义：
 *  高优先级队列中的消息总是先于低优先级队列被处理，行为可复现、可测试。
 * ============================================================================
 */
#ifndef RIBBED_FLOOR_RTOS_SIM_H_
#define RIBBED_FLOOR_RTOS_SIM_H_

#include <cstdint>
#include <cstddef>
#include <array>
#include <cassert>
#include <functional>
#include <string>
#include <vector>

#include "hal.h"   /* CpuFreq 主频档位 */

namespace rfds {

/* 任务优先级定义 */
enum TaskPriority : uint8_t {
    TASK_PRIO_SYSTEM_MONITOR = 0,
    TASK_PRIO_DATA_STORAGE   = 1,
    TASK_PRIO_RESULT_PACK    = 2,
    TASK_PRIO_PARAM_PARSE    = 3,
    TASK_PRIO_STRUCT_CALC    = 4,
};

/* ------------------------------------------------------------------------ */
/* 定长环形消息队列（对应 FreeRTOS xQueueCreate(queue_len, item_size)）        */
/*  满时 send 返回 false（模拟 errQUEUE_FULL，队列长度为 8）        */
/* ------------------------------------------------------------------------ */
template <typename T, size_t N>
class TaskQueue {
public:
    TaskQueue() : head_(0), tail_(0), count_(0) {}

    bool send(const T& item) {
        if (count_ >= N) return false;           /* 队列满 */
        buffer_[tail_] = item;
        tail_ = (tail_ + 1) % N;
        ++count_;
        return true;
    }

    bool receive(T& item) {
        if (count_ == 0) return false;           /* 队列空 */
        item = buffer_[head_];
        head_ = (head_ + 1) % N;
        --count_;
        return true;
    }

    bool empty() const { return count_ == 0; }
    size_t count() const { return count_; }
    static constexpr size_t capacity() { return N; }

private:
    std::array<T, N> buffer_;
    size_t head_;
    size_t tail_;
    size_t count_;
};

/* ------------------------------------------------------------------------ */
/* 简单内存池          */
/* ------------------------------------------------------------------------ */
constexpr size_t kMemoryPoolBlockSize = 64;    /* 每块 64B */
constexpr size_t kMemoryPoolBlockCount = 32;   /* 共 2KB   */

class FixedMemoryPool {
public:
    FixedMemoryPool() {
        free_list_.reserve(kMemoryPoolBlockCount);
        for (size_t i = 0; i < kMemoryPoolBlockCount; ++i) {
            free_list_.push_back(i);
        }
    }
    /* 分配一块；返回块下标，池满返回 -1 */
    int allocate() {
        if (free_list_.empty()) return -1;
        int idx = free_list_.back();
        free_list_.pop_back();
        return idx;
    }
    void release(int idx) {
        if (idx >= 0 && idx < static_cast<int>(kMemoryPoolBlockCount))
            free_list_.push_back(static_cast<size_t>(idx));
    }
    double usage_ratio() const {
        return static_cast<double>(kMemoryPoolBlockCount - free_list_.size()) /
               static_cast<double>(kMemoryPoolBlockCount);
    }

private:
    std::vector<size_t> free_list_;
};

/* ------------------------------------------------------------------------ */
/* 系统监控任务（监控温度/内存占用，>=80% 触发存储任务）          */
/* ------------------------------------------------------------------------ */
struct SystemHealthReport {
    double memory_usage_ratio;
    double estimated_power_mw;
    CpuFreq current_freq;
    uint32_t uptime_ms;
    bool abnormal;
    std::string note;
    SystemHealthReport() : memory_usage_ratio(0.0), estimated_power_mw(0.0),
        uptime_ms(0), abnormal(false) {}
};

}  /* namespace rfds */

#endif  /* RIBBED_FLOOR_RTOS_SIM_H_ */
