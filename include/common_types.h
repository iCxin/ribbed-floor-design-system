/**
 * ============================================================================
 *  水工钢筋混凝土肋形楼盖辅助设计系统 V1.0
 *  common_types.h —— 全系统公共类型定义
 * ============================================================================
 *  对应第三章 3.2.1 系统功能架构：
 *    本头文件定义五个核心模块（参数解析 / 结构计算 / 配筋优化 / 结果封装 /
 *    数据存储与管理）共用的错误码、数据结构与规范常量。
 *
 *  设计原则（对应）：
 *    轻量化 —— 全部使用定长基础类型，禁用异常抛出路径（返回错误码）；
 *    规范适配 —— 所有规范限值集中于 kRegulation 命名空间，便于切换
 *                SL 191-2008 / GB 50010-2010 校验标准。
 * ============================================================================
 */
#ifndef RIBBED_FLOOR_COMMON_TYPES_H_
#define RIBBED_FLOOR_COMMON_TYPES_H_

#include <cstdint>
#include <string>
#include <vector>

namespace rfds {

/* ------------------------------------------------------------------------ */
/* 系统级版本与标识                                                           */
/* ------------------------------------------------------------------------ */
constexpr const char* kSystemName = "Hydraulic Ribbed-Floor Auxiliary Design System";
constexpr const char* kSystemVersion = "1.0";
constexpr uint16_t kDeviceAddressSelf = 0x01;   /* 系统 CAN/串口 地址 */
constexpr uint16_t kDeviceAddrMin = 0x02;       /* 外部设备地址范围下限 */
constexpr uint16_t kDeviceAddrMax = 0xFF;       /* 外部设备地址范围上限 */

/* ------------------------------------------------------------------------ */
/* 功能码定义（对应协议 0x01 参数上行 / 0x81 结果下行）                        */
/* ------------------------------------------------------------------------ */
enum FunctionCode : uint8_t {
    FC_CONCRETE_SLAB_PARAM = 0x01,  /* 楼盖设计参数（外部设备 -> 系统） */
    FC_BEAM_ELEMENT_PARAM  = 0x02,  /* 肋梁设计参数（预留扩展）        */
    FC_RESULT_PACKAGE      = 0x81,  /* 计算结果输出（系统 -> 外部设备）*/
    FC_NACK_RESPONSE       = 0x8F,  /* 否定应答                        */
    FC_ACK_RESPONSE        = 0x90,  /* 肯定应答                        */
};

/* ------------------------------------------------------------------------ */
/* 参数解析模块错误码（对应参数解析模块）                          */
/* ------------------------------------------------------------------------ */
enum class ParameterErrorCode : uint8_t {
    SUCCESS = 0x00,
    FRAME_LOSS = 0x01,               /* 帧丢失：超时未检测到帧尾          */
    CRC_ERROR = 0x02,                /* CRC16 校验失败，数据畸变          */
    INVALID_DEVICE_ADDRESS = 0x03,   /* 设备地址非法                      */
    INVALID_FUNCTION_CODE = 0x04,    /* 功能码不支持                      */
    DATA_DECODING_ERROR = 0x05,      /* IEEE754 解码 / 位偏移错误         */
    MISSING_DATA_FIELD = 0x06,       /* 载荷长度不足，字段缺失            */
    INVALID_DATA_FORMAT = 0x07,      /* 数值非法（NaN / 负尺寸等）        */
    STRENGTH_GRADE_OUT_OF_RANGE = 0x08,  /* 混凝土强度等级超出 C20~C80    */
    REBAR_RATIO_OUT_OF_RANGE = 0x09,     /* 配筋率低于限值或超出上限      */
    ASPECT_RATIO_INVALID = 0x0A,     /* 长宽比与板类型逻辑冲突            */
    UNSUPPORTED_FUNCTION_CODE = 0x0B,
    BUFFER_TOO_SMALL = 0x0C,
    INVALID_PROTOCOL_FORMAT = 0x0D,  /* 数据长度字段与实际载荷不符        */
    LOAD_VALUE_OUT_OF_RANGE = 0x0E,  /* 荷载值超出规范取值范围            */
    THICKNESS_OUT_OF_RANGE = 0x0F,   /* 板厚超出规范最小厚度要求          */
    UNKNOWN_ERROR = 0x63,
};

/* 错误结构体：贯穿参数解析全流程 */
struct ParameterError {
    ParameterErrorCode code;
    std::string message;
    ParameterError() : code(ParameterErrorCode::UNKNOWN_ERROR), message("未知错误") {}
    ParameterError(ParameterErrorCode c, const std::string& msg) : code(c), message(msg) {}
    bool ok() const { return code == ParameterErrorCode::SUCCESS; }
};

/* ------------------------------------------------------------------------ */
/* 规范数据库常量（对应"规范参数数据表"）                          */
/*  取值依据：《水工混凝土结构设计规范》SL 191-2008、                        */
/*            《混凝土结构设计规范》GB 50010-2010                            */
/* ------------------------------------------------------------------------ */
namespace kRegulation {

/* --- 混凝土强度等级适用范围 --- */
constexpr int kConcreteGradeMin = 20;   /* C20 */
constexpr int kConcreteGradeMax = 80;   /* C80 */

/* --- 配筋率限值 --- */
constexpr double kMinRebarRatio = 0.002;     /* 最小配筋率 0.2%（SL 191-2008 受弯构件） */
constexpr double kMaxRebarRatio = 0.025;     /* 最大配筋率（避免超筋破坏）              */

/* --- 板厚与跨度 --- */
constexpr double kMinSlabThicknessM = 0.08;  /* 水工楼盖最小板厚 80 mm */
constexpr double kMaxSlabThicknessM = 0.60;  /* 常规肋形楼盖最大板厚   */
constexpr double kMinSpanM = 0.5;            /* 最小计算跨度 0.5 m     */
constexpr double kMaxSpanM = 12.0;           /* 最大计算跨度 12 m      */

/* --- 长宽比逻辑校验 --- */
/* SL 191-2008：长边/短边 <= 2 时应按双向板计算；2~3 时宜按双向板；
 * > 3 时可按单向板计算（沿短边方向布置受力钢筋）。 */
constexpr double kAspectRatioTwoWayMax = 3.0;   /* 双向板长宽比上限  */
constexpr double kAspectRatioOneWayMin = 2.0;   /* 单向板长宽比下限  */

/* --- 荷载取值（kPa） --- */
constexpr double kDeadLoadMin_kPa = 0.5;
constexpr double kDeadLoadMax_kPa = 25.0;
constexpr double kLiveLoadMin_kPa = 0.0;
constexpr double kLiveLoadMax_kPa = 30.0;   /* 水工泵房楼面活荷载上限 */

/* --- 挠度与裂缝 --- */
constexpr double kAllowableDeflectionRatio = 1.0 / 250.0;  /* 允许挠跨比 l0/250 */
constexpr double kAllowableCrackWidthMM = 0.30;            /* 长期处于水下的环境类别取 0.3 mm */
constexpr int    kSolverMaxIterations = 10;                /* 算法容错：最大迭代次数 */

/* --- 荷载持续时间增大系数查找表 --- */
/* 行：荷载持续时间分级 0:<1h 1:1h~1d 2:1d~1w 3:1w~1m 4:>1m
 * 列：受拉钢筋配筋率分级 0:<0.2% 1:0.2%~0.4% 2:0.4%~0.6% 3:0.6%~0.8% 4:>0.8% */
constexpr double kDeflectionIncreaseFactor[5][5] = {
    {1.4, 1.6, 1.8, 2.0, 2.2},
    {1.3, 1.5, 1.7, 1.9, 2.1},
    {1.2, 1.4, 1.6, 1.8, 2.0},
    {1.1, 1.3, 1.5, 1.7, 1.9},
    {1.0, 1.2, 1.4, 1.6, 1.8},
};

}  /* namespace kRegulation */

/* ------------------------------------------------------------------------ */
/* 楼盖类型与支座边界枚举                                                     */
/* ------------------------------------------------------------------------ */
enum SlabType { SLAB_ONE_WAY = 0, SLAB_TWO_WAY = 1 };

/* 板四边支座类型（对应边界条件处理） */
enum EdgeSupport {
    EDGE_SIMPLY_SUPPORTED = 0,  /* 简支：约束竖向位移，释放转角 */
    EDGE_FIXED = 1,             /* 固定：约束竖向位移与转角     */
    EDGE_FREE = 2               /* 自由边                       */
};

/* 四边支座配置（水工肋形楼盖典型为四边简支于肋梁上） */
struct EdgeSupportConfig {
    EdgeSupport edge_x_min;  /* x = 0 边      */
    EdgeSupport edge_x_max;  /* x = Lx 边     */
    EdgeSupport edge_y_min;  /* y = 0 边      */
    EdgeSupport edge_y_max;  /* y = Ly 边     */
    EdgeSupportConfig()
        : edge_x_min(EDGE_SIMPLY_SUPPORTED), edge_x_max(EDGE_SIMPLY_SUPPORTED),
          edge_y_min(EDGE_SIMPLY_SUPPORTED), edge_y_max(EDGE_SIMPLY_SUPPORTED) {}
};

/* ------------------------------------------------------------------------ */
/* 标准化设计参数结构体                                                       */
/*  参数解析模块的输出；结构计算模块的输入。                                  */
/*  对应接口 DesignParam：Parse_Param(uint8_t*, uint16_t, ...)     */
/* ------------------------------------------------------------------------ */
struct DesignParam {
    uint8_t  device_address;        /* 源设备地址                       */
    uint8_t  function_code;         /* 功能码                           */
    uint32_t frame_timestamp;       /* 帧内时间戳标识（可追溯性）       */

    int    concrete_grade;          /* 混凝土强度等级数值（20~80）      */
    double fc_design_MPa;           /* 混凝土抗压强度设计值 (MPa)       */
    double ft_design_MPa;           /* 混凝土抗拉强度设计值 (MPa)       */
    double slab_span_m;             /* 楼盖长边跨度 (m)，X 方向         */
    double slab_width_m;            /* 楼盖短边宽度 (m)，Y 方向         */
    double slab_thickness_m;        /* 板厚 (m)                         */
    double rebar_ratio;             /* 初始（建议）配筋率               */
    int    slab_type;               /* 0: 单向板, 1: 双向板             */
    double dead_load_kPa;           /* 均布恒荷载标准值 (kPa)           */
    double live_load_kPa;           /* 均布活荷载标准值 (kPa)           */
    int    load_duration_class;     /* 荷载持续时间分级 0~4             */
    EdgeSupportConfig supports;     /* 四边支座条件                     */

    DesignParam()
        : device_address(0), function_code(0), frame_timestamp(0),
          concrete_grade(0), fc_design_MPa(0.0), ft_design_MPa(0.0),
          slab_span_m(0.0), slab_width_m(0.0), slab_thickness_m(0.0),
          rebar_ratio(0.0), slab_type(1),
          dead_load_kPa(0.0), live_load_kPa(0.0), load_duration_class(4) {}
};

/* ------------------------------------------------------------------------ */
/* 结构计算结果（对应结构计算模块输出）                            */
/* ------------------------------------------------------------------------ */
struct DeflectionCheck {
    bool passed;
    double short_term_deflection_m;      /* 短期挠度 (m)               */
    double long_term_deflection_m;       /* 长期挠度 = 短期 * 增大系数 */
    double deflection_increase_factor;   /* 荷载长期作用增大系数       */
    double allowable_deflection_m;       /* 允许挠度 = l0/250          */
    std::string message;
    DeflectionCheck() : passed(false), short_term_deflection_m(0.0),
        long_term_deflection_m(0.0), deflection_increase_factor(1.0),
        allowable_deflection_m(0.0) {}
};

struct CrackCheck {
    bool passed;
    double max_crack_width_mm;           /* 最大裂缝宽度 (mm)          */
    double allowable_crack_width_mm;     /* 允许裂缝宽度 (mm)          */
    std::string message;
    CrackCheck() : passed(false), max_crack_width_mm(0.0),
        allowable_crack_width_mm(0.0) {}
};

/* 控制截面内力（沿 1 m 宽板带，单位 kN·m/m）
 * 双向板按 X、Y 两个方向板带分别提取弯矩包络；单向板仅 X 向为主受力方向。 */
struct ControlSectionForce {
    double midspan_moment_x_kNm;   /* X 向板带跨中最大正弯矩（板底受拉） */
    double support_moment_x_kNm;   /* X 向板带支座最大负弯矩（板顶受拉） */
    double midspan_moment_y_kNm;   /* Y 向板带跨中最大正弯矩             */
    double support_moment_y_kNm;   /* Y 向板带支座最大负弯矩             */
    double max_shear_kN;           /* 支座边缘最大剪力 (kN/m)            */
    ControlSectionForce() : midspan_moment_x_kNm(0.0), support_moment_x_kNm(0.0),
        midspan_moment_y_kNm(0.0), support_moment_y_kNm(0.0), max_shear_kN(0.0) {}
};

struct CalcResult {
    int    num_nodes_x;
    int    num_nodes_y;
    int    total_dofs;
    double max_node_displacement_m;      /* 节点最大竖向位移 (m)       */

    ControlSectionForce forces;
    DeflectionCheck     deflection_check;
    CrackCheck          crack_check;

    uint32_t elapsed_ticks;              /* 计算耗时（模拟系统节拍）   */
    bool     success;
    CalcResult() : num_nodes_x(0), num_nodes_y(0), total_dofs(0),
        max_node_displacement_m(0.0), elapsed_ticks(0), success(false) {}
};

/* ------------------------------------------------------------------------ */
/* 配筋优化结果（对应配筋优化模块输出）                            */
/* ------------------------------------------------------------------------ */
struct RebarScheme {
    int    bar_diameter_mm;       /* 钢筋直径 (mm)                    */
    double bar_spacing_mm;        /* 钢筋间距 (mm)                    */
    double area_mm2_per_m;        /* 每米板宽配筋面积 (mm^2/m)        */
    double moment_capacity_kNm;   /* 该方案板带抗弯承载力 (kN·m/m)    */
    double steel_stress_MPa;      /* 设计荷载下钢筋应力 (MPa)         */
    double crack_width_mm;        /* 对应裂缝宽度 (mm)                */
    RebarScheme() : bar_diameter_mm(0), bar_spacing_mm(0.0),
        area_mm2_per_m(0.0), moment_capacity_kNm(0.0),
        steel_stress_MPa(0.0), crack_width_mm(0.0) {}
};

struct OptResult {
    uint32_t design_number;           /* 唯一设计编号                       */
    uint32_t timestamp;               /* 结果生成时间戳                     */

    RebarScheme bottom_x;             /* X 向板底受力筋（跨中）             */
    RebarScheme bottom_y;             /* Y 向板底分布/受力筋                */
    RebarScheme top_x;                /* X 向支座板顶筋                     */
    RebarScheme top_y;                /* Y 向支座板顶筋                     */

    double total_rebar_ratio;         /* 全截面配筋率                       */
    double steel_weight_kg;           /* 钢筋总重量估算 (kg)                */
    double safety_factor;             /* 承载力安全储备 Mu/M                */

    bool   all_checks_passed;         /* 承载力 / 裂缝 / 挠度 全部通过      */
    int    iterations_used;           /* 实际迭代次数                       */
    bool   constructive_adjusted;     /* 是否被构造措施强制修正             */
    std::string summary;              /* 人类可读结论                       */

    OptResult() : design_number(0), timestamp(0), total_rebar_ratio(0.0),
        steel_weight_kg(0.0), safety_factor(0.0), all_checks_passed(false),
        iterations_used(0), constructive_adjusted(false) {}
};

/* ------------------------------------------------------------------------ */
/* 存储数据类型（对应数据存储与管理模块）                          */
/* ------------------------------------------------------------------------ */
enum StoreType : uint8_t {
    STORE_DESIGN_PARAM = 1,   /* 设计输入参数   */
    STORE_CALC_RESULT  = 2,   /* 结构计算结果   */
    STORE_OPT_RESULT   = 3,   /* 配筋优化方案   */
    STORE_PACKED_FRAME = 4,   /* 封装输出帧     */
};

/* 通用二进制负载：各模块经 Store_Data 接口传递的数据 */
using ByteVector = std::vector<uint8_t>;

}  /* namespace rfds */

#endif  /* RIBBED_FLOOR_COMMON_TYPES_H_ */
