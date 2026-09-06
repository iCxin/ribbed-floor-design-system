/**
 * ============================================================================
 *  水工钢筋混凝土肋形楼盖辅助设计系统 V1.0
 *  param_parser.h —— 模块一：参数解析模块
 * ============================================================================
 *  2.1 节参数解析模块、6.1 节工作流程：
 *
 *    1. 物理层数据接收与帧同步     synchronize_frame()
 *    2. 循环冗余校验与载荷提取     extract_and_validate_packet()
 *    3. 多级解码与数值映射         decode_and_map_parameters()
 *    4. 规范阈值边界检查           perform_threshold_checks()
 *
 *  对外接口：
 *    Parse_Param(uint8_t *data, uint16_t len, DesignParam *param)
 *  解析成功发送 ACK，失败发送 NACK（携带错误码）。
 * ============================================================================
 */
#ifndef RIBBED_FLOOR_PARAM_PARSER_H_
#define RIBBED_FLOOR_PARAM_PARSER_H_

#include <string>
#include <vector>

#include "common_types.h"
#include "protocol.h"

namespace rfds {

class ParameterParser {
public:
    ParameterParser() = default;

    /* 接口：Parse_Param —— 完整四步解析流水线。
     * 成功：填充 design_param 并经 UART 回 ACK，返回 true；
     * 失败：last_error 保留错误码，并回 NACK，返回 false。           */
    bool Parse_Param(const uint8_t* data, uint16_t len, DesignParam* design_param);

    /* 分步接口（供测试与流程剖析使用，对应四个步骤） */
    bool decode_and_map_parameters(const std::vector<uint8_t>& payload,
                                   uint8_t function_code,
                                   DesignParam& out, ParameterError& error);
    bool perform_threshold_checks(const DesignParam& params, ParameterError& error);

    /* 最近一次错误 */
    const ParameterError& last_error() const { return last_error_; }

private:
    /* 混凝土强度等级查表：等级数值 -> 抗压/抗拉强度设计值（MPa，SL 191-2008） */
    static bool query_concrete_strength(int grade, double* fc, double* ft);

    ParameterError last_error_{ParameterErrorCode::SUCCESS, "操作成功"};
};

}  /* namespace rfds */

#endif  /* RIBBED_FLOOR_PARAM_PARSER_H_ */
