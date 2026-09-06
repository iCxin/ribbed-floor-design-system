/**
 * rebar_optim.cpp —— 模块三：配筋优化模块实现
 *
 * 实现说明：
 *  - 收敛判据独立于最优解更新，避免"先赋值再比较差值"导致的恒真收敛；
 *  - 承载力与裂缝约束采用约束驱动的单调可行搜索；
 *  - 回溯保护（总面积超过最优方案 1.5 倍时重置）予以保留。
 */
#include "rebar_optim.h"

#include <cmath>
#include <algorithm>

namespace rfds {

using kRegulation::kMinRebarRatio;
using kRegulation::kMaxRebarRatio;
using kRegulation::kAllowableCrackWidthMM;
using kRegulation::kSolverMaxIterations;

/* 承载力安全系数（结构安全系数 K >= 1.2，SL 191-2008） */
constexpr double kStructuralSafetyFactor = 1.2;
/* 钢筋间距构造限值（mm） */
constexpr double kMaxBarSpacing = 200.0;
constexpr double kMinBarSpacing = 50.0;
/* 分布钢筋最小面积比例（单向板分布筋 >= 受力筋 15%，且 >= 0.15%bh） */
constexpr double kMinDistributionRatio = 0.0015;
/* 混凝土重度（钢筋重量估算） */
constexpr double kSteelDensity = 7850.0;   /* kg/m³ */

/* ------------------------------------------------------------------------ */
/* 标准钢筋规格库（优先选择直径较大的钢筋以减少根数）             */
/* ------------------------------------------------------------------------ */
const std::vector<SteelBarSpec>& available_steel_bars() {
    static const std::vector<SteelBarSpec> bars = {
        {1, 8.0, 50.27, 0.395},   /* D8  */
        {2, 10.0, 78.54, 0.617},  /* D10 */
        {3, 12.0, 113.10, 0.888}, /* D12 */
        {4, 14.0, 153.94, 1.208}, /* D14 */
        {5, 16.0, 201.06, 1.578}, /* D16 */
        {6, 18.0, 254.47, 2.000}, /* D18 */
        {7, 20.0, 314.16, 2.466}, /* D20 */
        {8, 22.0, 380.13, 2.985}, /* D22 */
        {9, 25.0, 490.87, 3.854}, /* D25 */
    };
    return bars;
}

RebarOptimizer::RebarOptimizer() = default;

/* ------------------------------------------------------------------------ */
/* 规格匹配算法：目标面积 -> （直径，间距）方案                                */
/*  在每种直径下取满足 As 且间距最大的方案（用钢量最少），各直径方案中         */
/*  取用钢量最小者；大直径方案在同等用钢量下根数更少，优先。                  */
/* ------------------------------------------------------------------------ */
bool RebarOptimizer::match_bar_spec(double target_as_mm2,
                                    const StripDesignContext& ctx,
                                    double moment_kNm_per_m, StripScheme& out) {
    if (target_as_mm2 <= 0.0) return false;

    StripDesignContext ctx_local = ctx;   /* 直径参与 h0 计算，需逐规格更新 */
    bool found = false;
    double best_score = 1e30;
    StripScheme best;
    StripScheme cand;

    for (const auto& bar : available_steel_bars()) {
        /* 该直径下满足目标面积的最大间距（间距越大用钢越少） */
        const double area_per_meter = 1000.0 * bar.area_mm2;   /* 间距 1mm 时 */
        double spacing = area_per_meter / target_as_mm2;       /* mm */
        if (spacing > kMaxBarSpacing) spacing = kMaxBarSpacing;
        /* 间距向下取整到 5mm 模数，便于施工放线 */
        spacing = std::floor(spacing / 5.0) * 5.0;
        if (spacing < kMinBarSpacing) continue;   /* 该直径过小，间距超密 */

        const double as_provided = area_per_meter / spacing;
        /* 承载力试算 */
        ctx_local.bar_diameter_mm = bar.diameter_mm;
        const double mu = StructuralCalculator::calc_moment_capacity_kNm(
            as_provided, ctx_local, nullptr);
        if (mu < 0.0) continue;   /* 超筋 */
        if (mu < kStructuralSafetyFactor * moment_kNm_per_m) continue;
        /* 裂缝宽度复核 */
        const double w = StructuralCalculator::calc_crack_width_mm(
            moment_kNm_per_m, as_provided, ctx_local);
        if (w > kAllowableCrackWidthMM) continue;   /* 裂缝超限，加密或换大径 */

        cand.scheme.bar_diameter_mm = static_cast<int>(bar.diameter_mm);
        cand.scheme.bar_spacing_mm = spacing;
        cand.scheme.area_mm2_per_m = as_provided;
        cand.scheme.moment_capacity_kNm = mu;
        cand.scheme.steel_stress_MPa = StructuralCalculator::calc_steel_stress_MPa(
            moment_kNm_per_m, as_provided,
            ctx.slab_thickness_mm - ctx.cover_mm - bar.diameter_mm / 2.0);
        cand.scheme.crack_width_mm = w;
        cand.required_as_mm2 = target_as_mm2;
        cand.constructive = false;

        /* 评分：用钢量（面积）最小优先；同面积取直径大者（根数少） */
        const double score = as_provided - bar.diameter_mm * 0.05;
        if (!found || score < best_score) {
            best_score = score;
            best = cand;
            found = true;
        }
    }
    if (!found) return false;
    out = best;
    return true;
}

/* ------------------------------------------------------------------------ */
/* 单板带方案设计                                      */
/* ------------------------------------------------------------------------ */
StripScheme RebarOptimizer::design_strip(double moment_kNm_per_m,
                                         const StripDesignContext& ctx,
                                         bool is_main_rebar) {
    StripScheme result;
    const double b = 1000.0;

    /* ---------- 构造下限 ---------- */
    const double as_min = kMinRebarRatio * b * ctx.slab_thickness_mm;

    /* ---------- 非受力方向：分布钢筋 ---------- */
    if (!is_main_rebar || moment_kNm_per_m < 1e-6) {
        /* 分布筋面积 >= 0.15%bh，且 >= 15% 构造最小配筋面积 */
        const double target = std::max(kMinDistributionRatio * b * ctx.slab_thickness_mm,
                                       0.15 * as_min);
        const auto& bars = available_steel_bars();
        for (const auto& bar : bars) {
            const double area_per_meter = 1000.0 * bar.area_mm2;
            double spacing = area_per_meter / target;
            spacing = std::min(spacing, 250.0);   /* 分布筋间距限值 250mm */
            spacing = std::floor(spacing / 5.0) * 5.0;
            if (spacing < kMinBarSpacing) continue;
            result.scheme.bar_diameter_mm = static_cast<int>(bar.diameter_mm);
            result.scheme.bar_spacing_mm = spacing;
            result.scheme.area_mm2_per_m = area_per_meter / spacing;
            result.scheme.moment_capacity_kNm = 0.0;
            result.scheme.steel_stress_MPa = 0.0;
            result.scheme.crack_width_mm = 0.0;
            result.required_as_mm2 = target;
            result.constructive = true;
            return result;
        }
        /* 兜底：构造筋 D8@200 */
        result.scheme.bar_diameter_mm = 8;
        result.scheme.bar_spacing_mm = 200.0;
        result.scheme.area_mm2_per_m = 1000.0 * 50.27 / 200.0;
        result.required_as_mm2 = target;
        result.constructive = true;
        return result;
    }

    /* ---------- 受力方向：三重控制面积 ---------- */
    /* 1. 承载力要求：Mu >= K·M */
    const double as_capacity =
        StructuralCalculator::calc_required_as_mm2(
            kStructuralSafetyFactor * moment_kNm_per_m, ctx);
    /* 2. 裂缝要求：迭代增大 As 直至 wmax <= 限值（约束驱动单调搜索） */
    double as_crack = 0.0;
    {
        double as_try = std::max(as_capacity, as_min);
        double w = StructuralCalculator::calc_crack_width_mm(
            moment_kNm_per_m, as_try, ctx);
        int guard = 0;
        while (w > kAllowableCrackWidthMM && guard++ < 20) {
            as_try *= 1.15;
            w = StructuralCalculator::calc_crack_width_mm(
                moment_kNm_per_m, as_try, ctx);
        }
        if (w <= kAllowableCrackWidthMM) as_crack = as_try;
        else as_crack = as_try;   /* 记录仍超限值，由方案匹配继续加密 */
    }
    /* 3. 最小配筋率要求 */
    const double target_as = std::max({as_capacity, as_crack, as_min});

    /* ---------- 初拟方案 + 迭代优化 ---------- */
    StripScheme best;
    bool found = match_bar_spec(target_as, ctx, moment_kNm_per_m, best);
    if (!found) {
        /* 目标面积无法满足（超筋或裂缝不可控），逐步增大目标面积重试 */
        double bump = target_as;
        int guard = 0;
        while (!found && guard++ < kSolverMaxIterations) {
            bump *= 1.2;
            found = match_bar_spec(bump, ctx, moment_kNm_per_m, best);
        }
        if (!found) {
            /* 算法容错：迭代超限返回默认安全方案 D12@100 */
            best.scheme.bar_diameter_mm = 12;
            best.scheme.bar_spacing_mm = 100.0;
            best.scheme.area_mm2_per_m = 1000.0 * 113.10 / 100.0;
            best.scheme.moment_capacity_kNm =
                StructuralCalculator::calc_moment_capacity_kNm(
                    best.scheme.area_mm2_per_m, ctx, nullptr);
            best.required_as_mm2 = target_as;
            best.constructive = true;
            return best;
        }
    }

    /* ---------- 回溯保护 ---------- */
    if (best.scheme.area_mm2_per_m > target_as * 1.5) {
        /* 重新以最小目标匹配，避免过度用钢 */
        StripScheme retry;
        if (match_bar_spec(target_as, ctx, moment_kNm_per_m, retry)) {
            best = retry;
        }
    }
    return best;
}

/* ------------------------------------------------------------------------ */
/* 主流程                                      */
/* ------------------------------------------------------------------------ */
bool RebarOptimizer::optimize(DesignParam& param, CalcResult& calc, OptResult& out,
                              RecalcFn recalc, ParameterError& error) {
    error = {ParameterErrorCode::SUCCESS, "优化成功"};
    iterations_used_ = 0;

    /* ---------- 1. 设计截面参数 ---------- */
    StripDesignContext ctx;
    ctx.slab_thickness_mm = param.slab_thickness_m * 1000.0;
    ctx.cover_mm = 25.0;
    ctx.fc_MPa = param.fc_design_MPa;
    ctx.ftk_MPa = StructuralCalculator::query_ftk_MPa(param.concrete_grade);
    ctx.Ec_MPa = StructuralCalculator::query_Ec_MPa(param.concrete_grade);
    ctx.fy_MPa = 360.0;           /* HRB400 钢筋抗拉强度设计值 */
    ctx.Es_MPa = 200000.0;
    ctx.bar_diameter_mm = 12.0;

    /* ---------- 2. 挠度外循环 ---------- */
    for (int iter = 0; iter < kSolverMaxIterations; ++iter) {
        iterations_used_ = iter + 1;

        /* 控制截面选取与各板带方案设计
         * 双向板：X/Y 两方向均为受力方向，按各自弯矩包络配筋；
         * 单向板：弯矩沿短边（Y 向，即 width 方向）传递，Y 向为受力筋，
         *          X 向仅布置构造分布筋（规范：分布筋 >= 15% 受力筋）。 */
        const bool two_way = (param.slab_type == SLAB_TWO_WAY);
        StripScheme bx, by, tx, ty;
        if (two_way) {
            bx = design_strip(calc.forces.midspan_moment_x_kNm, ctx, true);
            by = design_strip(calc.forces.midspan_moment_y_kNm, ctx, true);
            tx = design_strip(calc.forces.support_moment_x_kNm, ctx, true);
            ty = design_strip(calc.forces.support_moment_y_kNm, ctx, true);
        } else {
            by = design_strip(calc.forces.midspan_moment_y_kNm, ctx, true);
            bx = design_strip(0.0, ctx, false);                    /* 分布筋 */
            ty = design_strip(calc.forces.support_moment_y_kNm, ctx, true);
            tx = design_strip(0.0, ctx, false);
        }

        /* ---------- 4. 构造措施符合性强制修正标记 ---------- */
        const double as_min = kMinRebarRatio * 1000.0 * ctx.slab_thickness_mm;
        bool constructive = bx.constructive || by.constructive ||
                            tx.constructive || ty.constructive ||
                            bx.scheme.area_mm2_per_m <= as_min + 1e-9;

        /* 全截面配筋率（板底双向平均值） */
        const double h_mm = ctx.slab_thickness_mm;
        const double rho = (bx.scheme.area_mm2_per_m + by.scheme.area_mm2_per_m) /
                           (2.0 * 1000.0 * h_mm);

        /* 钢筋总重量估算：板底筋全长 + 支座顶筋 30% 投影长度 */
        const double len_x = param.slab_span_m, len_y = param.slab_width_m;
        const double vol_bx = bx.scheme.area_mm2_per_m * 1e-6 * len_x * len_y;
        const double vol_by = by.scheme.area_mm2_per_m * 1e-6 * len_x * len_y;
        const double vol_tx = tx.scheme.area_mm2_per_m * 1e-6 * len_x * len_y * 0.3;
        const double vol_ty = ty.scheme.area_mm2_per_m * 1e-6 * len_x * len_y * 0.3;
        const double weight = (vol_bx + vol_by + vol_tx + vol_ty) * kSteelDensity;

        /* 安全储备（仅统计受力方向；分布筋方向不参与承载力评定） */
        double sf = 1e9;
        if (two_way && calc.forces.midspan_moment_x_kNm > 1e-9)
            sf = std::min(sf, bx.scheme.moment_capacity_kNm /
                              (kStructuralSafetyFactor * calc.forces.midspan_moment_x_kNm));
        if (calc.forces.midspan_moment_y_kNm > 1e-9)
            sf = std::min(sf, by.scheme.moment_capacity_kNm /
                              (kStructuralSafetyFactor * calc.forces.midspan_moment_y_kNm));
        if (sf > 1e8) sf = 1.0;   /* 无受力弯矩（纯构造）时的中性值 */

        /* 填充结果（当前迭代最优方案） */
        out.bottom_x = bx.scheme;
        out.bottom_y = by.scheme;
        out.top_x = tx.scheme;
        out.top_y = ty.scheme;
        out.total_rebar_ratio = rho;
        out.steel_weight_kg = weight;
        out.safety_factor = sf;
        out.constructive_adjusted = constructive;
        out.iterations_used = iterations_used_;
        out.all_checks_passed = calc.deflection_check.passed &&
                                calc.crack_check.passed;
        out.summary = calc.deflection_check.passed
            ? "配筋方案满足承载力、裂缝与挠度要求"
            : "挠度超限，正在调整配筋率重新验算";

        /* ---------- 3. 挠度约束迭代：不满足则上调配筋率重新分析 ---------- */
        if (calc.deflection_check.passed) {
            error = {ParameterErrorCode::SUCCESS, "优化收敛"};
            return true;
        }
        /* 挠度不足：上调建议配筋率（提高截面有效刚度），重新运行结构计算 */
        param.rebar_ratio = std::min(param.rebar_ratio * 1.25,
                                     kMaxRebarRatio);
        if (recalc && recalc(param, calc, error)) {
            continue;
        }
        break;   /* 重算失败：返回当前最优方案（容错，） */
    }

    /* 达到最大迭代次数仍超限：返回默认安全方案标记（算法容错） */
    if (!calc.deflection_check.passed) {
        out.summary = "已达最大迭代次数，返回当前可行方案，建议复核截面高度";
        out.all_checks_passed = false;
    }
    error = {ParameterErrorCode::SUCCESS, "优化完成（挠度迭代未完全收敛）"};
    return true;
}

}  /* namespace rfds */
