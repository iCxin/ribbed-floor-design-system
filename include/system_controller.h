/**
 * ============================================================================
 *  水工钢筋混凝土肋形楼盖辅助设计系统 V1.0
 *  system_controller.h —— 系统整合控制器
 * ============================================================================
 *  2.2 节四层架构应用层、5.2 节任务调度设计：
 *
 *    结构计算任务  优先级 4（最高，独占 FPU）
 *    参数解析任务  优先级 3
 *    结果封装任务  优先级 2
 *    数据存储任务  优先级 1
 *    系统监控任务  优先级 0
 *
 *  模块间接口：
 *    Parse_Param / Calc_Callback（消息队列）/ 共享内存 0x24000000 + CRC32 /
 *    Package_Result / Store_Data
 * ============================================================================
 */
#ifndef RIBBED_FLOOR_SYSTEM_CONTROLLER_H_
#define RIBBED_FLOOR_SYSTEM_CONTROLLER_H_

#include <cstdint>
#include <string>
#include <vector>

#include "common_types.h"
#include "param_parser.h"
#include "struct_calc.h"
#include "rebar_optim.h"
#include "result_pack.h"
#include "data_storage.h"
#include "rtos_sim.h"
#include "hal.h"

namespace rfds {

/* 一次设计任务在流水线中的状态 */
enum class DesignStage {
    IDLE = 0,
    PARSED,          /* 参数解析完成 */
    CALCULATED,      /* 结构计算完成 */
    OPTIMIZED,       /* 配筋优化完成 */
    PACKAGED,        /* 结果封装并输出 */
    REJECTED         /* 参数非法，已回 NACK */
};

struct PipelineReport {
    DesignStage stage = DesignStage::IDLE;
    uint32_t design_number = 0;
    ParameterError error{ParameterErrorCode::SUCCESS, "操作成功"};

    DesignParam param;
    CalcResult calc;
    OptResult opt;

    std::vector<uint8_t> output_frame;    /* 经 UART 输出的完整二进制帧 */
    bool ack_sent = false;
    bool nack_sent = false;
    uint32_t elapsed_ms = 0;              /* 端到端耗时（模拟节拍） */
};

class EmbeddedSystemController {
public:
    EmbeddedSystemController();

    /* 上电初始化：外设自检、数据库加载 */
    void initialize();

    /* ------- 参数解析任务（优先级 3）：外部设备帧输入入口 ------- */
    bool submit_design_frame(const uint8_t* data, uint16_t len);

    /* ------- 结构计算任务（优先级 4）：处理计算队列 ------- */
    bool run_structural_calculation();

    /* ------- 结果封装任务（优先级 2）：输出二进制结果帧 ------- */
    bool run_result_packaging();

    /* ------- 数据存储任务（优先级 1）：后台存储队列 ------- */
    void run_storage_task();

    /* ------- 系统监控任务（优先级 0）：健康报告 ------- */
    SystemHealthReport run_monitor_task();

    /* 单次完整设计（外部设备一帧 -> 结果帧输出的同步流水线） */
    bool process_one_design(const uint8_t* data, uint16_t len,
                            PipelineReport& report);

    /* 最新一次流水线报告 */
    const PipelineReport& last_report() const { return last_report_; }

    /* 存储模块访问（历史记录调取） */
    StorageManagementModule& storage() { return storage_; }

    uint32_t next_design_number() { return ++design_counter_; }

private:
    /* ：配筋优化结果经共享内存传递（写入方 = 优化模块） */
    bool write_opt_to_shared_mem(const OptResult& opt);
    bool read_opt_from_shared_mem(OptResult& opt);

    /* 队列（计算队列长度 8、存储队列长度 16） */
    TaskQueue<DesignParam, 8> calc_queue_;
    TaskQueue<std::pair<StoreType, std::vector<uint8_t>>, 16> storage_queue_;

    ParameterParser parser_;
    StructuralCalculator calculator_;
    RebarOptimizer optimizer_;
    ResultPackager packager_;
    StorageManagementModule storage_;

    PipelineReport last_report_;
    uint32_t design_counter_ = 1000;      /* 唯一设计编号 */
    uint32_t boot_timestamp_ = 0;
    FixedMemoryPool mem_pool_;            /* 2KB 临时内存池 */
};

}  /* namespace rfds */

#endif  /* RIBBED_FLOOR_SYSTEM_CONTROLLER_H_ */
