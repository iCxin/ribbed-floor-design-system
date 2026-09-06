/**
 * ============================================================================
 *  水工钢筋混凝土肋形楼盖辅助设计系统 V1.0
 *  rebar_optim.h —— 模块三：配筋优化模块
 * ============================================================================
 *  2.1 节配筋优化模块、6.3 节工作流程：
 *
 *    1. 内力包络图生成与控制截面选取   select_control_sections()
 *    2. 初拟配筋方案与承载力试算       initial_reinforcement_design()
 *    3. 非线性约束迭代优化             iterate_optimization()
 *    4. 构造措施符合性强制修正         enforce_constructive_measures()
 *
 *  约束优化模型：
 *    目标函数：钢筋总截面面积 As 最小
 *    约束条件：Mu >= K·M（K=1.2 结构安全系数，SL 191-2008）
 *              wmax <= 0.3mm（裂缝宽度限值）
 *              挠度 <= l0/250（长期挠度限值）
 *              ρmin <= ρ <= ρmax，间距 50~200mm（构造要求）
 * ============================================================================
 */
#ifndef RIBBED_FLOOR_REBAR_OPTIM_H_
#define RIBBED_FLOOR_REBAR_OPTIM_H_

#include <functional>
#include <vector>

#include "common_types.h"
#include "struct_calc.h"

namespace rfds {

/* 标准钢筋规格库（D8~D25 九种常用直径） */
struct SteelBarSpec {
    int    id;
    double diameter_mm;
    double area_mm2;             /* 单根截面积 */
    double weight_kg_per_m;      /* 每米重量 */
};
const std::vector<SteelBarSpec>& available_steel_bars();

/* 单个板带的配筋方案设计结果 */
struct StripScheme {
    RebarScheme scheme;          /* 直径/间距/面积/承载力/应力/裂缝 */
    double required_as_mm2;      /* 计算所需钢筋面积 (mm²/m) */
    bool   constructive;         /* 是否由构造措施控制 */
    StripScheme() : required_as_mm2(0.0), constructive(false) {}
};

class RebarOptimizer {
public:
    /* 重算回调：挠度不满足时调整配筋率后重新运行结构分析
     * （对应"若不符合则自动调整配筋参数并重新验算"） */
    using RecalcFn = std::function<bool(DesignParam&, CalcResult&, ParameterError&)>;

    RebarOptimizer();

    /* 主流程：输入结构计算结果，输出最优配筋方案。
     * param 为可引用：挠度迭代时可能上调建议配筋率。 */
    bool optimize(DesignParam& param, CalcResult& calc, OptResult& out,
                  RecalcFn recalc, ParameterError& error);

    /* 单板带方案设计 */
    StripScheme design_strip(double moment_kNm_per_m,
                             const StripDesignContext& ctx,
                             bool is_main_rebar);

    /* 最近一次优化使用的迭代次数（含挠度外循环） */
    int last_iterations() const { return iterations_used_; }

private:
    /* 规格匹配：给定目标面积，在标准直径序列中优选（大直径优先减根数） */
    bool match_bar_spec(double target_as_mm2, const StripDesignContext& ctx,
                        double moment_kNm_per_m, StripScheme& out);

    int iterations_used_ = 0;
};

}  /* namespace rfds */

#endif  /* RIBBED_FLOOR_REBAR_OPTIM_H_ */
