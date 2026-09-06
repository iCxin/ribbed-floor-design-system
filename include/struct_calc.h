/**
 * ============================================================================
 *  水工钢筋混凝土肋形楼盖辅助设计系统 V1.0
 *  struct_calc.h —— 模块二：结构计算模块
 * ============================================================================
 *  结构计算主流程：
 *
 *    1. 网格自动剖分与刚度矩阵组装（ACM 四节点矩形薄板弯单元）
 *    2. 荷载向量生成与边界条件处理（一致节点荷载 + 支座约束消元）
 *    3. 乔列斯基分解求解线性方程组（LDL^T / LL^T，SPD 刚度矩阵）
 *    4. 长期刚度折减与挠度验算（荷载持续时间增大系数查表）
 *    5. 平均裂缝间距与宽度计算（粘结滑移理论，SL 191-2008 公式）
 *
 *  单元模型：ACM 四节点矩形薄板弯单元，每节点 w、∂w/∂x、∂w/∂y 三自由度；
 *  形函数由 12 项多项式基在节点插值条件下数值求逆得到，刚度矩阵经
 *  高斯积分组装，物理模型与弯矩/剪力恢复符合薄板理论。
 * ============================================================================
 */
#ifndef RIBBED_FLOOR_STRUCT_CALC_H_
#define RIBBED_FLOOR_STRUCT_CALC_H_

#include <vector>

#include "common_types.h"

namespace rfds {

/* ------------------------------------------------------------------------ */
/* 材料与截面参数（1 m 宽板带设计截面）                                        */
/* ------------------------------------------------------------------------ */
struct StripDesignContext {
    double slab_thickness_mm;      /* 板厚 h (mm)                       */
    double cover_mm;               /* 保护层厚度 (mm)                   */
    double fc_MPa;                 /* 混凝土抗压强度设计值              */
    double ftk_MPa;                /* 混凝土抗拉强度标准值              */
    double Ec_MPa;                 /* 混凝土弹性模量                    */
    double fy_MPa;                 /* 钢筋抗拉强度设计值（HRB400: 360） */
    double Es_MPa;                 /* 钢筋弹性模量 (2.0e5 MPa)          */
    double bar_diameter_mm;        /* 计算裂缝宽度用钢筋直径 deq        */
    StripDesignContext() : slab_thickness_mm(0.0), cover_mm(25.0), fc_MPa(0.0),
        ftk_MPa(0.0), Ec_MPa(0.0), fy_MPa(360.0), Es_MPa(200000.0),
        bar_diameter_mm(12.0) {}
};

class StructuralCalculator {
public:
    StructuralCalculator() = default;

    /* 主流程：由标准化设计参数完成全部分析与验算 */
    bool perform_structural_analysis(const DesignParam& param,
                                     CalcResult& result, ParameterError& error);

    /* ---------------- 共享工程公式（供配筋优化模块复用） ---------------- */
    /* 钢筋应力 σs = M / (0.87 h0 As)，M: kN·m/m，As: mm²/m，返回 MPa */
    static double calc_steel_stress_MPa(double moment_kNm_per_m,
                                        double as_mm2_per_m, double h0_mm);
    /* 最大裂缝宽度 wmax（GB 50010-2010 / SL 191-2008 粘结滑移公式，mm） */
    static double calc_crack_width_mm(double moment_kNm_per_m, double as_mm2_per_m,
                                      const StripDesignContext& ctx);
    /* 单筋矩形截面抗弯承载力 Mu（kN·m/m），并输出混凝土受压区高度 x (mm) */
    static double calc_moment_capacity_kNm(double as_mm2_per_m,
                                           const StripDesignContext& ctx,
                                           double* x_mm = nullptr);
    /* 满足弯矩 M 所需钢筋面积 As（mm²/m）；超筋时返回 -1 */
    static double calc_required_as_mm2(double moment_kNm_per_m,
                                       const StripDesignContext& ctx);
    /* 混凝土抗拉强度标准值 ftk（MPa），按强度等级查表 */
    static double query_ftk_MPa(int concrete_grade);
    /* 混凝土弹性模量 Ec（MPa），按强度等级查表 */
    static double query_Ec_MPa(int concrete_grade);

private:
    /* ---------------- FEM 内部构件 ---------------- */
    /* ACM 单元自然坐标 12 项多项式基及其各阶导数 */
    static double basis_value(int term, double xi, double eta);
    static double basis_dxi(int term, double xi, double eta);
    static double basis_deta(int term, double xi, double eta);
    static double basis_dxixi(int term, double xi, double eta);
    static double basis_detaeta(int term, double xi, double eta);
    static double basis_dxieta(int term, double xi, double eta);

    /* 12x12 矩阵求逆（高斯-约当消元，部分主元），用于形函数推导 */
    static bool invert_12x12(double A[12][12]);

    /* 密对称正定矩阵 Cholesky（LL^T）分解求解；失败返回 false（奇异） */
    static bool solve_cholesky(std::vector<double>& A, std::vector<double>& b,
                               int n, ParameterError& error);
};

}  /* namespace rfds */

#endif  /* RIBBED_FLOOR_STRUCT_CALC_H_ */
