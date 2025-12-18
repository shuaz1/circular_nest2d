#pragma once
#include "layout.h"
#include "candidate_points.h"
#include <vector>
#include <functional>

namespace nesting {

    // 高利用率圆形排样核心算法
    class CircleNesting {
    public:
        explicit CircleNesting(Layout& layout);

        // 圆形排样参数
        struct Parameters {
            // 圆直径优化参数
            double diameter_step_ratio = 0.005;  // 每次直径调整的步长比例（0.5%，更精细）
            double min_diameter_ratio = 0.6;      // 最小直径相对于理论下界的比例
            size_t diameter_optimization_iterations = 100; // 直径优化迭代次数（增加）
            bool use_binary_search = true;       // 使用二分查找优化直径

            // 紧凑化参数
            size_t compact_iterations = 500;      // 局部紧凑化迭代次数（大幅增加）
            double compact_step_size = 0.002;     // 紧凑化步长（相对于直径，更小更精细）
            double compact_radius_search = 0.15;  // 紧凑化搜索半径（相对于直径）
            bool use_simulated_annealing = true; // 使用模拟退火优化
            double sa_initial_temp = 0.1;         // 模拟退火初始温度
            double sa_cooling_rate = 0.95;        // 模拟退火冷却率

            // NFP候选点参数
            size_t max_candidate_points = 2000;   // 最大候选点数（大幅增加）
            bool use_radial_candidates = true;    // 是否使用径向候选点生成
            size_t radial_layers = 20;            // 径向层数（增加）
            size_t angles_per_layer = 32;         // 每层角度数（增加）
            bool prioritize_compact_positions = true; // 优先选择紧凑位置

            // 放置策略参数
            bool use_smart_ordering = true;       // 使用智能排序（考虑形状复杂度）
            bool try_all_rotations = true;        // 为每个零件尝试所有旋转角度
            size_t max_rotation_attempts = 4;     // 最大旋转尝试次数

            // 旋转精度（度数）
            double rotation_precision = 1.0;      // 默认1度精度

            // 阵列排样模式参数（可选）
            bool use_array_mode = false;          // 是否启用阵列排样模式（默认关闭，保持兼容）
            double array_pitch_x = 0.0;           // 阵列X方向间距（0表示自动：零件宽度+间隙）
            double array_pitch_y = 0.0;           // 阵列Y方向间距（0表示自动：零件高度+间隙）
            double array_margin = 0.0;            // 阵列最外圈距离圆边的安全距离
        };

        void set_parameters(const Parameters& params);

        // 执行高利用率圆形排样
        // 返回是否成功达到目标利用率（默认0.92，优秀级别）
        bool optimize(double target_utilization = 0.92, 
                     volatile bool* requestQuit = nullptr,
                     std::function<void(const Layout&)> progress_callback = nullptr);

        // 获取当前最优利用率
        double get_best_utilization() const { return best_utilization_; }

        // 获取当前圆直径
        double get_current_diameter() const;

    private:
        Layout& layout_;
        Parameters params_;
        double best_utilization_;
        double current_diameter_;

        // 计算理论最小直径（基于总面积）
        double calculate_theoretical_min_diameter() const;

        // 优化圆直径：从理论下界开始，逐步缩小直到无法放置
        bool optimize_diameter(double target_utilization, volatile bool* requestQuit,
                              std::function<void(const Layout&)> progress_callback);

        // 阵列排样模式：在固定直径下生成规则阵列布局
        bool optimize_array_mode(double target_utilization, volatile bool* requestQuit,
                                 std::function<void(const Layout&)> progress_callback);

        // 在给定直径下尝试放置所有零件
        bool try_place_all_parts(double diameter, volatile bool* requestQuit,
                                 std::function<void(const Layout&)>* progress_callback = nullptr);

        // 基于NFP的精确放置（针对圆形优化）
        bool place_shape_with_nfp(size_t shape_idx, const std::vector<size_t>& placed_indices, 
                                 bool try_all_rotations = true);

        // 生成圆形优化的候选点
        std::vector<geo::Point_2> generate_circular_candidates(
            size_t shape_idx, 
            const std::vector<size_t>& placed_indices,
            const geo::Polygon_2& ifr);

        // 局部紧凑化：让已放置零件向空隙移动（增强版）
        void compact_layout(size_t iterations, volatile bool* requestQuit,
                           std::function<void(const Layout&)>* progress_callback = nullptr);

        // 智能排序：考虑面积、形状复杂度、长宽比
        std::vector<size_t> smart_order_parts() const;

        // 计算零件的形状复杂度（用于排序）
        double calculate_shape_complexity(size_t shape_idx) const;

        // 二分查找最优直径
        double binary_search_diameter(double min_diameter, double max_diameter, 
                                      double target_utilization, volatile bool* requestQuit,
                                      std::function<void(const Layout&)> progress_callback);

        // 计算当前布局的利用率
        double calculate_utilization() const;

        // 检查零件是否在圆内且无重叠
        bool is_valid_placement(size_t shape_idx, const std::vector<size_t>& placed_indices) const;

        // 更新圆直径并重建sheet
        void update_circle_diameter(double diameter);
    };

} // namespace nesting

