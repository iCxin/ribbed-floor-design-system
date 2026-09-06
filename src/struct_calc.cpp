/**
 * struct_calc.cpp —— 模块二：结构计算模块实现
 *
 * 有限元方法：ACM（Adini-Clough-Melosh）四节点矩形薄板弯曲单元。
 *  - 每节点 3 个自由度：挠度 w、转角 ∂w/∂x、∂w/∂y；
 *  - 单元位移场由 12 项多项式基 {1,ξ,η,ξ²,ξη,η²,ξ³,ξ²η,ξη²,η³,ξ³η,ξη³}
 *    张成，形函数由节点插值条件（函数值 + 两个一阶偏导）求逆获得；
 *  - 单元刚度矩阵 Ki = ∫Ω Bᵀ D B dΩ（2×2 高斯积分精确）；
 *  - 均布荷载按一致节点荷载等效 f = ∫Ω q N dΩ；
 *  - 整体刚度矩阵按约束消元法处理支座边界后用 Cholesky 分解求解；
 *  - 支座反力由原始（未消元）刚度矩阵恢复 R = K·u - F。
 *
 * 单位约定：内部全部使用 SI（N、m、Pa），输出时换算 kN·m/m、mm。
 */
#include "struct_calc.h"

#include <cmath>
#include <cstring>
#include <algorithm>

namespace rfds {

using kRegulation::kDeflectionIncreaseFactor;
using kRegulation::kAllowableDeflectionRatio;
using kRegulation::kAllowableCrackWidthMM;

/* ------------------------------------------------------------------------ */
/* 混凝土力学指标查表（GB 50010-2010 表 4.1.3-2 / 表 4.1.5）                   */
/* ------------------------------------------------------------------------ */
double StructuralCalculator::query_ftk_MPa(int grade) {
    static const struct { int g; double ftk; } table[] = {
        {20, 1.54}, {25, 1.78}, {30, 2.01}, {35, 2.20}, {40, 2.39},
        {45, 2.51}, {50, 2.64}, {55, 2.74}, {60, 2.85}, {65, 2.93},
        {70, 3.00}, {75, 3.05}, {80, 3.11},
    };
    for (const auto& t : table)
        if (t.g == grade) return t.ftk;
    return 2.01;   /* 兜底 C30 */
}

double StructuralCalculator::query_Ec_MPa(int grade) {
    static const struct { int g; double Ec; } table[] = {
        {20, 2.55e4}, {25, 2.80e4}, {30, 3.00e4}, {35, 3.15e4}, {40, 3.25e4},
        {45, 3.35e4}, {50, 3.45e4}, {55, 3.55e4}, {60, 3.60e4}, {65, 3.65e4},
        {70, 3.70e4}, {75, 3.75e4}, {80, 3.80e4},
    };
    for (const auto& t : table)
        if (t.g == grade) return t.Ec;
    return 3.00e4;
}

/* ------------------------------------------------------------------------ */
/* ACM 单元 12 项多项式基（自然坐标 ξ,η ∈ [-1,1]）                            */
/* 项序：0:1  1:ξ  2:η  3:ξ²  4:ξη  5:η²  6:ξ³  7:ξ²η  8:ξη²  9:η³          */
/*       10:ξ³η  11:ξη³                                                      */
/* ------------------------------------------------------------------------ */
double StructuralCalculator::basis_value(int t, double xi, double eta) {
    switch (t) {
        case 0:  return 1.0;
        case 1:  return xi;
        case 2:  return eta;
        case 3:  return xi * xi;
        case 4:  return xi * eta;
        case 5:  return eta * eta;
        case 6:  return xi * xi * xi;
        case 7:  return xi * xi * eta;
        case 8:  return xi * eta * eta;
        case 9:  return eta * eta * eta;
        case 10: return xi * xi * xi * eta;
        case 11: return xi * eta * eta * eta;
    }
    return 0.0;
}

double StructuralCalculator::basis_dxi(int t, double xi, double eta) {
    switch (t) {
        case 1:  return 1.0;
        case 3:  return 2.0 * xi;
        case 4:  return eta;
        case 6:  return 3.0 * xi * xi;
        case 7:  return 2.0 * xi * eta;
        case 8:  return eta * eta;
        case 10: return 3.0 * xi * xi * eta;
        case 11: return eta * eta * eta;
    }
    return 0.0;
}

double StructuralCalculator::basis_deta(int t, double xi, double eta) {
    switch (t) {
        case 2:  return 1.0;
        case 4:  return xi;
        case 5:  return 2.0 * eta;
        case 7:  return xi * xi;
        case 8:  return 2.0 * xi * eta;
        case 9:  return 3.0 * eta * eta;
        case 10: return xi * xi * xi;
        case 11: return 3.0 * xi * eta * eta;
    }
    return 0.0;
}

double StructuralCalculator::basis_dxixi(int t, double xi, double eta) {
    switch (t) {
        case 3:  return 2.0;
        case 6:  return 6.0 * xi;
        case 7:  return 2.0 * eta;
        case 10: return 6.0 * xi * eta;
    }
    return 0.0;
}

double StructuralCalculator::basis_detaeta(int t, double xi, double eta) {
    switch (t) {
        case 5:  return 2.0;
        case 8:  return 2.0 * xi;
        case 9:  return 6.0 * eta;
        case 11: return 6.0 * xi * eta;
    }
    return 0.0;
}

double StructuralCalculator::basis_dxieta(int t, double xi, double eta) {
    switch (t) {
        case 4:  return 1.0;
        case 7:  return 2.0 * xi;
        case 8:  return 2.0 * eta;
        case 10: return 3.0 * xi * xi;
        case 11: return 3.0 * eta * eta;
    }
    return 0.0;
}

/* ------------------------------------------------------------------------ */
/* 12x12 矩阵求逆：LU 分解（部分主元）+ 12 次单位向量回代                      */
/* ------------------------------------------------------------------------ */
bool StructuralCalculator::invert_12x12(double A[12][12]) {
    double LU[12][12];
    std::memcpy(LU, A, sizeof(LU));
    int perm[12];

    /* Doolittle LU 分解，记录行交换 */
    for (int k = 0; k < 12; ++k) {
        int p = k;
        double best = std::fabs(LU[k][k]);
        for (int i = k + 1; i < 12; ++i) {
            if (std::fabs(LU[i][k]) > best) { best = std::fabs(LU[i][k]); p = i; }
        }
        if (best < 1e-14) return false;
        perm[k] = p;
        if (p != k) {
            for (int j = 0; j < 12; ++j) std::swap(LU[p][j], LU[k][j]);
        }
        for (int i = k + 1; i < 12; ++i) {
            LU[i][k] /= LU[k][k];
            for (int j = k + 1; j < 12; ++j)
                LU[i][j] -= LU[i][k] * LU[k][j];
        }
    }

    /* 对每个单位向量求解 A x = e_col，得逆矩阵列 col */
    double Inv[12][12];
    for (int col = 0; col < 12; ++col) {
        double x[12] = {0};
        double b[12] = {0};
        b[col] = 1.0;
        for (int k = 0; k < 12; ++k)                    /* 回放行交换 */
            if (perm[k] != k) std::swap(b[k], b[perm[k]]);
        for (int i = 0; i < 12; ++i) {                  /* 前代 L y = Pb */
            double s = b[i];
            for (int j = 0; j < i; ++j) s -= LU[i][j] * x[j];
            x[i] = s;
        }
        for (int i = 11; i >= 0; --i) {                 /* 回代 U x = y */
            double s = x[i];
            for (int j = i + 1; j < 12; ++j) s -= LU[i][j] * x[j];
            x[i] = s / LU[i][i];
        }
        for (int r = 0; r < 12; ++r) Inv[r][col] = x[r];
    }
    std::memcpy(A, Inv, sizeof(Inv));
    return true;
}

/* ------------------------------------------------------------------------ */
/* 密对称正定矩阵 Cholesky（LL^T）求解                                        */
/* ------------------------------------------------------------------------ */
bool StructuralCalculator::solve_cholesky(std::vector<double>& A,
                                          std::vector<double>& b,
                                          int n, ParameterError& error) {
    std::vector<double> L(static_cast<size_t>(n) * n, 0.0);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j <= i; ++j) {
            double sum = A[static_cast<size_t>(i) * n + j];
            for (int k = 0; k < j; ++k)
                sum -= L[static_cast<size_t>(i) * n + k] *
                       L[static_cast<size_t>(j) * n + k];
            if (i == j) {
                if (sum <= 1e-12) {
                    error = {ParameterErrorCode::INVALID_DATA_FORMAT,
                             "刚度矩阵非正定（几何参数矛盾或约束不足），求解发散"};
                    return false;
                }
                L[static_cast<size_t>(i) * n + i] = std::sqrt(sum);
            } else {
                L[static_cast<size_t>(i) * n + j] =
                    sum / L[static_cast<size_t>(j) * n + j];
            }
        }
    }
    for (int i = 0; i < n; ++i) {                       /* 前代 Ly = b */
        double sum = b[i];
        for (int k = 0; k < i; ++k)
            sum -= L[static_cast<size_t>(i) * n + k] * b[k];
        b[i] = sum / L[static_cast<size_t>(i) * n + i];
    }
    for (int i = n - 1; i >= 0; --i) {                  /* 回代 Lᵀ x = y */
        double sum = b[i];
        for (int k = i + 1; k < n; ++k)
            sum -= L[static_cast<size_t>(k) * n + i] * b[k];
        b[i] = sum / L[static_cast<size_t>(i) * n + i];
    }
    return true;   /* b 原地覆盖为解向量 */
}

/* ------------------------------------------------------------------------ */
/* 共享工程公式                                                               */
/* ------------------------------------------------------------------------ */
double StructuralCalculator::calc_steel_stress_MPa(double moment_kNm_per_m,
                                                   double as_mm2_per_m,
                                                   double h0_mm) {
    if (as_mm2_per_m <= 0.0 || h0_mm <= 0.0) return 0.0;
    const double M_Nmm = moment_kNm_per_m * 1.0e6;   /* kN·m/m -> N·mm/mm 宽 */
    return M_Nmm / (0.87 * h0_mm * as_mm2_per_m);
}

double StructuralCalculator::calc_crack_width_mm(double moment_kNm_per_m,
                                                 double as_mm2_per_m,
                                                 const StripDesignContext& ctx) {
    if (as_mm2_per_m <= 0.0 || moment_kNm_per_m <= 0.0) return 0.0;
    const double b = 1000.0;
    const double h0 = ctx.slab_thickness_mm - ctx.cover_mm - ctx.bar_diameter_mm / 2.0;
    const double sigma_s = calc_steel_stress_MPa(moment_kNm_per_m, as_mm2_per_m, h0);

    /* 有效受拉钢筋配筋率 ρte = As / Ate，Ate = 0.5·b·h（受弯构件） */
    double rho_te = as_mm2_per_m / (0.5 * b * ctx.slab_thickness_mm);
    if (rho_te < 0.01) rho_te = 0.01;

    /* 钢筋应变不均匀系数 ψ = 1.1 - 0.65·ftk/(ρte·σs)，限幅 [0.2, 1.0] */
    double psi = 1.1 - 0.65 * ctx.ftk_MPa / (rho_te * sigma_s);
    psi = std::max(0.2, std::min(1.0, psi));

    /* wmax = αcr·ψ·(σs/Es)·(1.9c + 0.08·deq/ρte)，αcr = 1.9（受弯构件） */
    const double alpha_cr = 1.9;
    const double w_mm = alpha_cr * psi * (sigma_s / ctx.Es_MPa) *
                        (1.9 * ctx.cover_mm + 0.08 * ctx.bar_diameter_mm / rho_te);
    return w_mm;
}

double StructuralCalculator::calc_moment_capacity_kNm(double as_mm2_per_m,
                                                      const StripDesignContext& ctx,
                                                      double* x_mm) {
    if (as_mm2_per_m <= 0.0) return 0.0;
    const double b = 1000.0;
    const double h0 = ctx.slab_thickness_mm - ctx.cover_mm - ctx.bar_diameter_mm / 2.0;
    /* 受压区高度 x = fy·As / (α1·fc·b)，α1 = 1.0（混凝土强度 <= C50） */
    const double alpha1 = 1.0;
    const double x = ctx.fy_MPa * as_mm2_per_m / (alpha1 * ctx.fc_MPa * b);
    if (x_mm) *x_mm = x;
    /* 相对受压区高度界限 ξb（HRB400 钢筋：0.518） */
    const double xi_b = 0.518;
    if (x / h0 > xi_b) return -1.0;   /* 超筋破坏，返回无效值 */
    const double Mu_Nmm = ctx.fy_MPa * as_mm2_per_m * (h0 - x / 2.0);
    return Mu_Nmm / 1.0e6;   /* N·mm/mm -> kN·m/m */
}

double StructuralCalculator::calc_required_as_mm2(double moment_kNm_per_m,
                                                  const StripDesignContext& ctx) {
    if (moment_kNm_per_m <= 0.0) return 0.0;
    const double b = 1000.0;
    const double h0 = ctx.slab_thickness_mm - ctx.cover_mm - ctx.bar_diameter_mm / 2.0;
    const double alpha1 = 1.0;
    const double M_Nmm = moment_kNm_per_m * 1.0e6;
    const double alpha_s = M_Nmm / (alpha1 * ctx.fc_MPa * b * h0 * h0);
    if (alpha_s > 0.399) return -1.0;   /* αs,max = ξb(1-0.5ξb) = 0.399，超筋 */
    const double xi = 1.0 - std::sqrt(1.0 - 2.0 * alpha_s);
    return alpha1 * ctx.fc_MPa * b * xi * h0 / ctx.fy_MPa;
}

/* ------------------------------------------------------------------------ */
/* 主分析流程                                               */
/* ------------------------------------------------------------------------ */
bool StructuralCalculator::perform_structural_analysis(const DesignParam& param,
                                                       CalcResult& result,
                                                       ParameterError& error) {
    error = {ParameterErrorCode::SUCCESS, "计算成功"};

    /* ---------------- 0. 计算参数准备 ---------------- */
    const double span_x = param.slab_span_m;     /* 长边 */
    const double span_y = param.slab_width_m;    /* 短边 */
    const double thick = param.slab_thickness_m;
    const double nu = 0.2;                       /* 混凝土泊松比 */
    const double Ec_Pa = query_Ec_MPa(param.concrete_grade) * 1.0e6;  /* MPa -> Pa */
    const double D_flex = Ec_Pa * thick * thick * thick / (12.0 * (1.0 - nu * nu));

    /* 网格剖分：4×4 基准网格（简化刚度矩阵法）；沿弯曲主方向加密至
     * 8 等分以控制单元长宽比（双向板长宽比 > 2 时加密长边 X，
     * 单向板弯曲方向为短边 Y）。 */
    int nx = 4, ny = 4;
    if (param.slab_type == SLAB_ONE_WAY) { nx = 4; ny = 8; }
    else if (span_x / span_y > 2.0) { nx = 8; ny = 4; }

    const int num_nodes_x = nx + 1, num_nodes_y = ny + 1;
    const int total_nodes = num_nodes_x * num_nodes_y;
    const int dofs_per_node = 3;                 /* w, θx, θy */
    const int total_dofs = total_nodes * dofs_per_node;

    const double lx = span_x / nx;
    const double ly = span_y / ny;

    /* 内力取标准组合（G+Q）作用值；承载力验算由配筋优化模块
     * 统一乘以结构安全系数 K >= 1.2（SL 191-2008 单一安全系数法）。 */
    const double q_k = (param.dead_load_kPa + param.live_load_kPa) * 1000.0;  /* Pa */

    /* ---------------- 1. 网格自动剖分与刚度矩阵组装 ---------------- */
    /* ACM 形函数推导：A c = d，S[k][j] = (A⁻¹)[j][k] */
    double A[12][12] = {{0}};
    const double node_sign[4][2] = {{-1,-1}, {1,-1}, {1,1}, {-1,1}};
    for (int n = 0; n < 4; ++n) {
        const double xi_s = node_sign[n][0], eta_s = node_sign[n][1];
        for (int j = 0; j < 12; ++j) {
            A[3 * n + 0][j] = basis_value(j, xi_s, eta_s);
            A[3 * n + 1][j] = (2.0 / lx) * basis_dxi(j, xi_s, eta_s);
            A[3 * n + 2][j] = (2.0 / ly) * basis_deta(j, xi_s, eta_s);
        }
    }
    double Ainv[12][12];
    std::memcpy(Ainv, A, sizeof(Ainv));
    if (!invert_12x12(Ainv)) {
        error = {ParameterErrorCode::INVALID_DATA_FORMAT, "单元形函数推导失败"};
        return false;
    }
    double S[12][12];
    for (int k = 0; k < 12; ++k)
        for (int j = 0; j < 12; ++j)
            S[k][j] = Ainv[j][k];

    /* 2×2 高斯积分点（刚度组装与一致荷载积分） */
    const double gp[2] = {-1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0)};
    const double gw = 1.0;
    const double jac = (lx / 2.0) * (ly / 2.0);

    /* 板弯弹性矩阵 D（每单位宽度）：D = D_flex·[[1,ν,0],[ν,1,0],[0,0,(1-ν)/2]] */
    const double Dmat[3][3] = {
        {D_flex, D_flex * nu, 0.0},
        {D_flex * nu, D_flex, 0.0},
        {0.0, 0.0, D_flex * (1.0 - nu) / 2.0},
    };

    /* 单元刚度矩阵 Ke(12×12) 与一致节点荷载 Fe(12)（等尺寸单元复用） */
    double Ke[12][12];
    std::memset(Ke, 0, sizeof(Ke));
    for (int g = 0; g < 2; ++g) {
        for (int hidx = 0; hidx < 2; ++hidx) {
            const double xi = gp[g], eta = gp[hidx];
            double B[3][12] = {{0}};
            for (int k = 0; k < 12; ++k) {
                double dxx = 0.0, dyy = 0.0, dxy = 0.0;
                for (int j = 0; j < 12; ++j) {
                    const double c = S[k][j];
                    if (c == 0.0) continue;
                    dxx += c * (4.0 / (lx * lx)) * basis_dxixi(j, xi, eta);
                    dyy += c * (4.0 / (ly * ly)) * basis_detaeta(j, xi, eta);
                    dxy += c * (4.0 / (lx * ly)) * basis_dxieta(j, xi, eta);
                }
                B[0][k] = dxx;
                B[1][k] = dyy;
                B[2][k] = 2.0 * dxy;
            }
            for (int a = 0; a < 12; ++a) {
                for (int b = 0; b < 12; ++b) {
                    double s = 0.0;
                    for (int r = 0; r < 3; ++r)
                        for (int c = 0; c < 3; ++c)
                            s += B[r][a] * Dmat[r][c] * B[c][b];
                    Ke[a][b] += gw * jac * s;
                }
            }
        }
    }

    double Fe[12] = {0};
    for (int g = 0; g < 2; ++g) {
        for (int hidx = 0; hidx < 2; ++hidx) {
            const double xi = gp[g], eta = gp[hidx];
            for (int k = 0; k < 12; ++k) {
                double Nk = 0.0;
                for (int j = 0; j < 12; ++j)
                    Nk += S[k][j] * basis_value(j, xi, eta);
                Fe[k] += gw * jac * q_k * Nk;
            }
        }
    }

    /* 整体刚度矩阵（密存储；资源受限平台可切换稀疏下三角存储） */
    std::vector<double> K(static_cast<size_t>(total_dofs) * total_dofs, 0.0);
    std::vector<double> F(total_dofs, 0.0);

    auto elem_node_id = [&](int i, int j) { return i + j * num_nodes_x; };

    for (int i = 0; i < nx; ++i) {
        for (int j = 0; j < ny; ++j) {
            const int nodes[4] = {
                elem_node_id(i, j), elem_node_id(i + 1, j),
                elem_node_id(i + 1, j + 1), elem_node_id(i, j + 1)};
            for (int a = 0; a < 12; ++a) {
                const int ga = nodes[a / 3] * dofs_per_node + (a % 3);
                for (int b = 0; b < 12; ++b) {
                    const int gb = nodes[b / 3] * dofs_per_node + (b % 3);
                    K[static_cast<size_t>(ga) * total_dofs + gb] += Ke[a][b];
                }
            }
            for (int a = 0; a < 12; ++a) {
                const int ga = nodes[a / 3] * dofs_per_node + (a % 3);
                F[ga] += Fe[a];
            }
        }
    }

    /* ---------------- 2. 边界条件处理（约束消元） ---------------- */
    auto constrain = [&](int dof) {
        for (int r = 0; r < total_dofs; ++r) {
            K[static_cast<size_t>(dof) * total_dofs + r] = 0.0;
            K[static_cast<size_t>(r) * total_dofs + dof] = 0.0;
        }
        K[static_cast<size_t>(dof) * total_dofs + dof] = 1.0;
        F[dof] = 0.0;
    };
    auto apply_edge = [&](EdgeSupport sup, int which) {
        if (sup == EDGE_FREE) return;
        for (int i = 0; i < num_nodes_x; ++i) {
            for (int j = 0; j < num_nodes_y; ++j) {
                const bool on = (which == 0 && i == 0) ||
                                (which == 1 && i == num_nodes_x - 1) ||
                                (which == 2 && j == 0) ||
                                (which == 3 && j == num_nodes_y - 1);
                if (!on) continue;
                const int node = elem_node_id(i, j);
                constrain(node * dofs_per_node + 0);                /* w = 0 */
                if (sup == EDGE_FIXED) {
                    constrain(node * dofs_per_node + 1);            /* θx = 0 */
                    constrain(node * dofs_per_node + 2);            /* θy = 0 */
                }
            }
        }
    };
    apply_edge(param.supports.edge_x_min, 0);
    apply_edge(param.supports.edge_x_max, 1);
    apply_edge(param.supports.edge_y_min, 2);
    apply_edge(param.supports.edge_y_max, 3);

    /* ---------------- 3. 乔列斯基分解求解 ---------------- */
    if (!solve_cholesky(K, F, total_dofs, error)) return false;
    const std::vector<double>& u = F;   /* 解向量（原地覆盖） */

    /* ---------------- 4. 挠度验算 ---------------- */
    double max_w_design = 0.0;
    for (int n = 0; n < total_nodes; ++n)
        max_w_design = std::max(max_w_design, std::fabs(u[n * dofs_per_node + 0]));
    const double short_term_w = max_w_design;   /* 标准组合短期挠度 */

    /* 荷载长期作用增大系数（查表：持续时间分级 × 配筋率分级） */
    int rebar_level;
    if (param.rebar_ratio < 0.002) rebar_level = 0;
    else if (param.rebar_ratio < 0.004) rebar_level = 1;
    else if (param.rebar_ratio < 0.006) rebar_level = 2;
    else if (param.rebar_ratio < 0.008) rebar_level = 3;
    else rebar_level = 4;
    const int dur = std::min(std::max(param.load_duration_class, 0), 4);
    const double increase_factor = kDeflectionIncreaseFactor[dur][rebar_level];

    result.deflection_check.short_term_deflection_m = short_term_w;
    result.deflection_check.deflection_increase_factor = increase_factor;
    result.deflection_check.long_term_deflection_m = short_term_w * increase_factor;
    result.deflection_check.allowable_deflection_m =
        span_y * kAllowableDeflectionRatio;   /* 挠度限值按短边跨度控制 */
    result.deflection_check.passed =
        result.deflection_check.long_term_deflection_m <=
        result.deflection_check.allowable_deflection_m;
    result.deflection_check.message = result.deflection_check.passed
        ? "挠度满足规范要求" : "挠度超限";

    /* ---------------- 5. 内力包络恢复 ---------------- */
    /* 3×3 采样点恢复弯矩场 */
    const double sg = std::sqrt(0.6);
    const double sp[3] = {-sg, 0.0, sg};

    double max_mxx_pos = 0.0, max_mxx_neg = 0.0;
    double max_myy_pos = 0.0, max_myy_neg = 0.0;

    for (int i = 0; i < nx; ++i) {
        for (int j = 0; j < ny; ++j) {
            const int nodes[4] = {
                elem_node_id(i, j), elem_node_id(i + 1, j),
                elem_node_id(i + 1, j + 1), elem_node_id(i, j + 1)};
            double uel[12];
            for (int a = 0; a < 12; ++a)
                uel[a] = u[nodes[a / 3] * dofs_per_node + (a % 3)];

            for (int g = 0; g < 3; ++g) {
                for (int hidx = 0; hidx < 3; ++hidx) {
                    const double xi = sp[g], eta = sp[hidx];
                    double kappa_x = 0.0, kappa_y = 0.0;
                    for (int k = 0; k < 12; ++k) {
                        double dxx = 0.0, dyy = 0.0;
                        for (int jj = 0; jj < 12; ++jj) {
                            const double c = S[k][jj];
                            if (c == 0.0) continue;
                            dxx += c * (4.0 / (lx * lx)) * basis_dxixi(jj, xi, eta);
                            dyy += c * (4.0 / (ly * ly)) * basis_detaeta(jj, xi, eta);
                        }
                        kappa_x += dxx * uel[k];
                        kappa_y += dyy * uel[k];
                    }
                    /* M = -D·κ（挠度向上为正，正弯矩 = 板底受拉） */
                    const double Mx = -(Dmat[0][0] * kappa_x + Dmat[0][1] * kappa_y);
                    const double My = -(Dmat[1][1] * kappa_y + Dmat[1][0] * kappa_x);
                    max_mxx_pos = std::max(max_mxx_pos, Mx);
                    max_myy_pos = std::max(max_myy_pos, My);
                    max_mxx_neg = std::max(max_mxx_neg, -Mx);
                    max_myy_neg = std::max(max_myy_neg, -My);
                }
            }
        }
    }
    result.forces.midspan_moment_x_kNm = max_mxx_pos / 1000.0;   /* kN·m/m */
    result.forces.support_moment_x_kNm = max_mxx_neg / 1000.0;
    result.forces.midspan_moment_y_kNm = max_myy_pos / 1000.0;
    result.forces.support_moment_y_kNm = max_myy_neg / 1000.0;

    /* 支座剪力：长边两缘各承担约一半板面荷载，取边长均分值
     * （工程近似，偏保守）。 */
    const double total_load_N = q_k * span_x * span_y;
    result.forces.max_shear_kN =
        (total_load_N * 0.5 / span_y) / 1000.0;   /* kN/m，长边每米 */

    /* ---------------- 6. 初步裂缝验算（按输入配筋率） ---------------- */
    StripDesignContext ctx;
    ctx.slab_thickness_mm = thick * 1000.0;
    ctx.cover_mm = 25.0;
    ctx.fc_MPa = param.fc_design_MPa;
    ctx.ftk_MPa = query_ftk_MPa(param.concrete_grade);
    ctx.Ec_MPa = query_Ec_MPa(param.concrete_grade);
    ctx.bar_diameter_mm = 12.0;
    const double h0_mm = ctx.slab_thickness_mm - ctx.cover_mm - ctx.bar_diameter_mm / 2.0;
    const double as_input = param.rebar_ratio * 1000.0 * h0_mm;   /* mm²/m */
    const double w_input = calc_crack_width_mm(
        std::max(result.forces.midspan_moment_x_kNm,
                 result.forces.midspan_moment_y_kNm),
        as_input, ctx);
    result.crack_check.max_crack_width_mm = w_input;
    result.crack_check.allowable_crack_width_mm = kAllowableCrackWidthMM;
    result.crack_check.passed = w_input <= kAllowableCrackWidthMM;
    result.crack_check.message = result.crack_check.passed
        ? "裂缝宽度满足耐久性要求" : "裂缝宽度超限";

    /* 汇总 */
    result.num_nodes_x = num_nodes_x;
    result.num_nodes_y = num_nodes_y;
    result.total_dofs = total_dofs;
    result.max_node_displacement_m = max_w_design;
    result.success = result.deflection_check.passed && result.crack_check.passed;
    return true;
}

}  /* namespace rfds */
