#include "circle_nesting.h"
#include "nesting.h"
#include "algorithm.h"
#include <CGAL/boolean_set_operations_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/number_utils.h>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <random>
#include <set>

namespace nesting {

// Robust conversion from CGAL::FT to double using intervals
static double safe_to_double(const geo::FT& v) {
    auto I = CGAL::to_interval(v);
    double a = I.first;
    double b = I.second;
    if (!std::isfinite(a) || !std::isfinite(b)) {
        return 0.0;
    }
    return 0.5 * (a + b);
}

CircleNesting::CircleNesting(Layout& layout)
    : layout_(layout), best_utilization_(0.0), current_diameter_(0.0) {
    if (layout_.sheets.empty() || layout_.sheets[0].type != Sheet::ShapeType::Circle) {
        throw std::runtime_error("CircleNesting requires a circular sheet");
    }
    current_diameter_ = safe_to_double(layout_.sheets[0].diameter);
}

void CircleNesting::set_parameters(const Parameters& params) {
    params_ = params;
}

double CircleNesting::calculate_theoretical_min_diameter() const {
    // 理论最小直径：sqrt(4 * 总面积 / PI)
    // 考虑利用率目标，实际最小直径 = sqrt(4 * 总面积 / (PI * target_utilization))
    double total_area = safe_to_double(layout_.area);
    double min_diameter = std::sqrt(4.0 * total_area / (Sheet::kPi * 0.92)); // 假设92%利用率
    return min_diameter;
}

void CircleNesting::update_circle_diameter(double diameter) {
    current_diameter_ = diameter;
    layout_.sheets[0] = Sheet::make_circle(geo::FT(diameter), layout_.sheets[0].segments);
    layout_.sheets[0].diameter = geo::FT(diameter);
}

double CircleNesting::calculate_utilization() const {
    if (layout_.sheet_parts.empty() || layout_.sheet_parts[0].empty()) {
        return 0.0;
    }

    try {
        // 计算圆形板材面积
        double radius = safe_to_double(layout_.sheets[0].diameter) / 2.0;
        double circle_area = Sheet::kPi * radius * radius;
        
        if (circle_area <= 0) {
            return 0.0;
        }

        // 获取圆形板材多边形
        geo::Polygon_with_holes_2 sheet_pwh = layout_.sheets[0].sheet;
        
        // 使用 Polygon_set_2 计算所有已放置零件的并集面积
        // 注意：Polygon_set_2 的 join 操作会自动处理重叠（去重叠），确保重叠部分只计算一次
        // 这符合约束2：刀头之间不能重叠（如果放置时遵守了约束，这里不应该有重叠，但计算时仍然去重叠）
        geo::Polygon_set_2 ps(geo::traits);
        
        for (const auto& shape : layout_.sheet_parts[0]) {
            try {
                // 约束1：只计算在圆内的部分（刀头不能超出板材）
                // 计算零件与板材的交集（只计算在圆内的部分）
                std::vector<geo::Polygon_with_holes_2> inters;
                try {
                    CGAL::intersection(shape.transformed, sheet_pwh, std::back_inserter(inters));
                }
                catch (...) {
                    inters.clear();
                }
                
                // 如果交集为空，说明零件不在圆内，跳过（遵守约束1）
                if (inters.empty()) {
                    continue;
                }
                
                // 约束2：将交集加入多边形集合（自动处理重叠，确保重叠部分只计算一次）
                for (const auto& p : inters) {
                    try {
                        ps.join(p.outer_boundary());
                    }
                    catch (...) {
                        // 忽略错误，继续处理下一个
                    }
                }
            }
            catch (...) {
                // 忽略单个零件的错误，继续处理下一个
            }
        }
        
        // 计算并集总面积
        std::vector<geo::Polygon_with_holes_2> unions;
        double union_area = 0.0;
        try {
            ps.polygons_with_holes(std::back_inserter(unions));
        }
        catch (...) {
            unions.clear();
        }
        
        for (const auto& p : unions) {
            try {
                union_area += safe_to_double(geo::pwh_area(p));
            } 
            catch (...) {
                // 忽略错误
            }
        }
        
        // 利用率 = 实际覆盖面积（并集，去重叠）/ 圆形板材面积
        // 注意：这个计算方式与CAD可能不同：
        // - CAD可能计算的是：选择的图形总面积 / 圆面积（理论值，不考虑实际位置和重叠）
        // - 程序计算的是：实际在圆内的覆盖面积（并集，去重叠）/ 圆面积（实际值）
        double utilization = union_area / circle_area;
        if (utilization < 0) utilization = 0;
        if (utilization > 1) utilization = 1;
        
        return utilization;
    }
    catch (...) {
        return 0.0;
    }
}

// ============================================================================
// 核心约束检查函数：确保零件不超出圆形板材且不重叠
// 这是代码的底线，所有放置操作都必须通过此检查
// 
// 约束1：刀头不能超出板材（零件必须完全在圆形板材内）
// 约束2：刀头之间不能重叠（零件之间不能有任何重叠）
// ============================================================================
bool CircleNesting::is_valid_placement(size_t shape_idx, const std::vector<size_t>& placed_indices) const {
    if (shape_idx >= layout_.sheet_parts[0].size()) {
        return false;
    }

    const auto& shape = layout_.sheet_parts[0][shape_idx];
    
    // ========== 约束1：严格检查是否完全在圆内（不能超出圆形板材） ==========
    try {
        double radius = safe_to_double(layout_.sheets[0].diameter) / 2.0;
        double center_x = radius;
        double center_y = radius;
        double radius_sq = radius * radius;
        const double tolerance = 1e-6; // 允许的数值误差

        // 获取圆形板材多边形
        geo::Polygon_with_holes_2 sheet_pwh = layout_.sheets[0].sheet;
        
        // 方法1：检查所有顶点是否在圆内（快速检查）
        for (auto v = shape.transformed.outer_boundary().vertices_begin();
             v != shape.transformed.outer_boundary().vertices_end(); ++v) {
            double dx = safe_to_double(v->x()) - center_x;
            double dy = safe_to_double(v->y()) - center_y;
            double dist_sq = dx * dx + dy * dy;
            if (dist_sq > radius_sq + tolerance) {
                return false; // 顶点超出圆形
            }
        }
        
        // 方法2：检查所有边上的点（采样检查，确保边不超出）
        for (auto e = shape.transformed.outer_boundary().edges_begin();
             e != shape.transformed.outer_boundary().edges_end(); ++e) {
            // 在边的中点采样检查
            auto source = e->source();
            auto target = e->target();
            double mid_x = safe_to_double((source.x() + target.x()) / 2.0);
            double mid_y = safe_to_double((source.y() + target.y()) / 2.0);
            double dx = mid_x - center_x;
            double dy = mid_y - center_y;
            double dist_sq = dx * dx + dy * dy;
            if (dist_sq > radius_sq + tolerance) {
                return false; // 边超出圆形
            }
        }
        
        // 方法3：使用CGAL布尔运算，检查多边形是否完全在圆内
        // 计算多边形与圆的交集，如果交集面积等于多边形面积，说明完全在圆内
        try {
            std::vector<geo::Polygon_with_holes_2> intersection;
            CGAL::intersection(shape.transformed, sheet_pwh, std::back_inserter(intersection));
            
            // 计算交集总面积
            double intersection_area = 0.0;
            for (const auto& poly : intersection) {
                try {
                    intersection_area += safe_to_double(geo::pwh_area(poly));
                }
                catch (...) {
                    // 忽略错误
                }
            }
            
            // 计算多边形总面积
            double shape_area = safe_to_double(geo::pwh_area(shape.transformed));
            
            // 如果交集面积小于多边形面积（超过容差），说明有部分超出圆形
            if (intersection_area < shape_area - tolerance) {
                return false;
            }
        }
        catch (...) {
            // 如果布尔运算失败，使用保守策略：只检查顶点和边（已在上面完成）
            // 如果顶点和边都在圆内，通常整个多边形也在圆内
        }
        
        // 检查孔（如果有）
        for (auto h = shape.transformed.holes_begin(); h != shape.transformed.holes_end(); ++h) {
            for (auto v = h->vertices_begin(); v != h->vertices_end(); ++v) {
                double dx = safe_to_double(v->x()) - center_x;
                double dy = safe_to_double(v->y()) - center_y;
                double dist_sq = dx * dx + dy * dy;
                if (dist_sq > radius_sq + tolerance) {
                    return false;
                }
            }
        }
    }
    catch (...) {
        return false;
    }

    // ========== 约束2：检查是否与已放置零件重叠（不能重叠） ==========
    // 优化：先用 bbox 快速排除，再用精确检测
    auto bbox_i = shape.transformed.bbox();
    
    for (size_t j : placed_indices) {
        if (j == shape_idx) continue;
        
        try {
            const auto& shape_j = layout_.sheet_parts[0][j];
            
            // 快速检测：bbox 不相交则必定不重叠
            auto bbox_j = shape_j.transformed.bbox();
            if (bbox_i.xmax() < bbox_j.xmin() || bbox_i.xmin() > bbox_j.xmax() ||
                bbox_i.ymax() < bbox_j.ymin() || bbox_i.ymin() > bbox_j.ymax()) {
                continue; // bbox 不相交，跳过精确检测
            }
            
            // 精确检测：CGAL 布尔运算
            geo::Polygon_set_2 ps1(geo::traits);
            ps1.insert(shape.transformed.outer_boundary());
            
            geo::Polygon_set_2 ps2(geo::traits);
            ps2.insert(shape_j.transformed.outer_boundary());
            
            ps1.intersection(ps2);
            
            if (!ps1.is_empty()) {
                std::vector<geo::Polygon_with_holes_2> intersection;
                intersection.reserve(4);
                ps1.polygons_with_holes(std::back_inserter(intersection));
                
                double overlap_area = 0.0;
                for (const auto& poly : intersection) {
                    overlap_area += safe_to_double(geo::pwh_area(poly));
                }
                if (overlap_area > 1e-6) {
                    return false;
                }
            }
        }
        catch (...) {
            return false;
        }
    }

    return true;
}

std::vector<geo::Point_2> CircleNesting::generate_circular_candidates(
    size_t shape_idx,
    const std::vector<size_t>& placed_indices,
    const geo::Polygon_2& ifr) {
    
    std::vector<geo::Point_2> candidates;
    
    if (shape_idx >= layout_.sheet_parts[0].size()) {
        return candidates;
    }

    try {
        const auto& shape = layout_.sheet_parts[0][shape_idx];
        double radius = safe_to_double(layout_.sheets[0].diameter) / 2.0;
        double center_x = radius;
        double center_y = radius;

        // 方法1：基于NFP的完美点（如果可用）
        if (!placed_indices.empty()) {
            CandidatePoints c(placed_indices.size());
            c.set_boundary(ifr);
            
            auto rotation_i = shape.get_rotation();
            for (size_t j : placed_indices) {
                const auto& shape_j = layout_.sheet_parts[0][j];
                auto rotation_j = shape_j.get_rotation();
                auto& nfp = comp_nfp(shape_j.base, rotation_j, shape_j.allowed_rotations,
                                    shape.base, rotation_i, shape.allowed_rotations, layout_);
                c.nfps.push_back(&nfp);
                c.translations.push_back(shape_j.get_translate());
                c.translate_x.push_back(shape_j.get_translate_double_x());
            }
            
            c.initialize();
            auto perfect_points = c.get_perfect_points();
            
            // 过滤：只保留在圆内的点
            for (const auto& pt : perfect_points) {
                double dx = safe_to_double(pt.x()) - center_x;
                double dy = safe_to_double(pt.y()) - center_y;
                double dist_sq = dx * dx + dy * dy;
                if (dist_sq <= radius * radius) {
                    candidates.push_back(pt);
                }
            }
        }

        // 方法2：径向候选点生成（如果启用）- 增强版
        if (params_.use_radial_candidates && candidates.size() < params_.max_candidate_points) {
            auto bbox = shape.transformed.bbox();
            double shape_width = safe_to_double(bbox.xmax() - bbox.xmin());
            double shape_height = safe_to_double(bbox.ymax() - bbox.ymin());
            double shape_diagonal = std::sqrt(shape_width * shape_width + shape_height * shape_height);
            
            // 生成径向候选点：从中心到边缘，多个角度（使用参数配置）
            size_t radial_layers = params_.radial_layers;
            size_t angles_per_layer = params_.angles_per_layer;
            
            // 优先生成靠近中心的候选点（更紧凑）
            for (size_t layer = 0; layer < radial_layers; ++layer) {
                // 非线性分布：中心区域更密集
                double layer_ratio = double(layer) / (radial_layers - 1);
                double r_ratio = 0.15 + 0.75 * (layer_ratio * layer_ratio); // 平方分布，中心更密集
                double r = radius * r_ratio;
                
                for (size_t a = 0; a < angles_per_layer; ++a) {
                    double angle = 2.0 * Sheet::kPi * double(a) / angles_per_layer;
                    // 添加小角度偏移以增加候选点密度
                    for (int offset = -1; offset <= 1; offset += 2) {
                        double angle_offset = angle + offset * (2.0 * Sheet::kPi / angles_per_layer / 4.0);
                        double x = center_x + r * std::cos(angle_offset) - shape_width / 2.0;
                        double y = center_y + r * std::sin(angle_offset) - shape_height / 2.0;
                        
                        // 确保在圆内
                        double max_dist = std::sqrt(
                            std::max((x + shape_width - center_x) * (x + shape_width - center_x),
                                    (x - center_x) * (x - center_x)) +
                            std::max((y + shape_height - center_y) * (y + shape_height - center_y),
                                    (y - center_y) * (y - center_y)));
                        
                        if (max_dist <= radius * 0.98) { // 留出安全边距
                            candidates.push_back(geo::Point_2(geo::FT(x), geo::FT(y)));
                        }
                        
                        if (candidates.size() >= params_.max_candidate_points) {
                            break;
                        }
                    }
                    if (candidates.size() >= params_.max_candidate_points) {
                        break;
                    }
                }
                if (candidates.size() >= params_.max_candidate_points) {
                    break;
                }
            }
        }

        // 方法3：IFR边界上的点
        if (candidates.size() < params_.max_candidate_points && !ifr.is_empty()) {
            for (auto v = ifr.vertices_begin(); v != ifr.vertices_end(); ++v) {
                double dx = safe_to_double(v->x()) - center_x;
                double dy = safe_to_double(v->y()) - center_y;
                double dist_sq = dx * dx + dy * dy;
                if (dist_sq <= radius * radius) {
                    candidates.push_back(*v);
                }
                if (candidates.size() >= params_.max_candidate_points) {
                    break;
                }
            }
        }
    }
    catch (...) {
        // 如果生成失败，返回空列表
    }

    return candidates;
}

bool CircleNesting::place_shape_with_nfp(size_t shape_idx, const std::vector<size_t>& placed_indices, 
                                         bool try_all_rotations) {
    if (shape_idx >= layout_.sheet_parts[0].size()) {
        return false;
    }

    auto& shape = layout_.sheet_parts[0][shape_idx];
    
    try {
        // 确保transformed是最新的
        shape.update();
        
        // 计算IFR（使用transformed，因为IFR需要知道当前旋转状态）
        auto ifr = geo::comp_ifr(layout_.sheets[0].sheet, shape.transformed);
        if (ifr.is_empty()) {
            return false;
        }

        // 生成候选点
        auto candidates = generate_circular_candidates(shape_idx, placed_indices, ifr);
        
        if (candidates.empty()) {
            return false;
        }

        // 尝试每个候选点，选择最紧凑的位置（最靠近中心）
        double radius = safe_to_double(layout_.sheets[0].diameter) / 2.0;
        double center_x = radius;
        double center_y = radius;
        
        geo::Point_2 best_point;
        uint32_t best_rotation = shape.get_rotation();
        double best_score = std::numeric_limits<double>::max();
        bool found = false;

        // 如果需要尝试所有旋转角度
        std::vector<uint32_t> rotations_to_try;
        if (try_all_rotations && params_.try_all_rotations) {
            // 尝试所有允许的旋转角度（如果allowed_rotations=4，则尝试0°, 90°, 180°, 270°）
            // 但不超过max_rotation_attempts限制（避免旋转次数过多）
            size_t max_rot = std::min(static_cast<size_t>(shape.allowed_rotations), 
                                     static_cast<size_t>(params_.max_rotation_attempts));
            for (uint32_t rot = 0; rot < max_rot; ++rot) {
                rotations_to_try.push_back(rot);
            }
        } else {
            rotations_to_try.push_back(shape.get_rotation());
        }

        uint32_t original_rotation = shape.get_rotation();
        
        for (uint32_t rot : rotations_to_try) {
            shape.set_rotation(rot);
            shape.update();
            
            // 重新计算IFR（因为旋转改变了形状）
            auto ifr_rotated = geo::comp_ifr(layout_.sheets[0].sheet, shape.transformed);
            if (ifr_rotated.is_empty()) {
                continue;
            }
            
            // 如果旋转了，重新生成候选点
            std::vector<geo::Point_2> candidates_for_rotation = candidates;
            if (rot != original_rotation) {
                candidates_for_rotation = generate_circular_candidates(shape_idx, placed_indices, ifr_rotated);
            }

            for (const auto& candidate : candidates_for_rotation) {
                // 临时放置
                auto old_x = shape.get_translate_ft_x();
                auto old_y = shape.get_translate_ft_y();
                
                shape.set_translate(candidate.x(), candidate.y());
                shape.update();

                // 检查是否有效
                if (is_valid_placement(shape_idx, placed_indices)) {
                    // 计算评分：距离中心 + 空隙利用率
                    double dx = safe_to_double(candidate.x()) - center_x;
                    double dy = safe_to_double(candidate.y()) - center_y;
                    double dist_sq = dx * dx + dy * dy;
                    
                    // 优先选择靠近中心的位置
                    double score = dist_sq;
                    
                    // 如果启用紧凑位置优先，进一步优化评分
                    if (params_.prioritize_compact_positions) {
                        // 计算到最近已放置零件的距离（鼓励填充空隙）
                        double min_dist_to_placed = std::numeric_limits<double>::max();
                        for (size_t j : placed_indices) {
                            auto bbox_j = layout_.sheet_parts[0][j].transformed.bbox();
                            double cx_j = safe_to_double(bbox_j.xmin() + bbox_j.xmax()) / 2.0;
                            double cy_j = safe_to_double(bbox_j.ymin() + bbox_j.ymax()) / 2.0;
                            double dist_to_j = std::sqrt((dx + center_x - cx_j) * (dx + center_x - cx_j) + 
                                                          (dy + center_y - cy_j) * (dy + center_y - cy_j));
                            min_dist_to_placed = std::min(min_dist_to_placed, dist_to_j);
                        }
                        // 鼓励填充小空隙
                        score = dist_sq * 0.7 + min_dist_to_placed * 0.3;
                    }
                    
                    if (score < best_score) {
                        best_score = score;
                        best_point = candidate;
                        best_rotation = rot;
                        found = true;
                    }
                }

                // 恢复
                shape.set_translate(old_x, old_y);
                shape.update();
            }
        }

        if (found) {
            shape.set_rotation(best_rotation);
            shape.set_translate(best_point.x(), best_point.y());
            shape.update();
            return true;
        } else {
            // 恢复原始旋转
            shape.set_rotation(original_rotation);
            shape.update();
        }

        return false;
    }
    catch (...) {
        return false;
    }
}

bool CircleNesting::try_place_all_parts(double diameter, volatile bool* requestQuit,
                                         std::function<void(const Layout&)>* progress_callback) {
    update_circle_diameter(diameter);

    // 重置所有零件位置：将未放置的零件放在板材外面（负坐标区域）
    // 这样在可视化时不会影响显示
    double radius = diameter / 2.0;
    double outside_offset = -radius * 2.0; // 放在板材外足够远的位置
    for (auto& shape : layout_.sheet_parts[0]) {
        shape.set_translate(geo::FT(outside_offset), geo::FT(outside_offset));
        shape.update();
    }

    // 智能排序：考虑面积、形状复杂度、长宽比
    std::vector<size_t> indices;
    if (params_.use_smart_ordering) {
        indices = smart_order_parts();
    } else {
        indices.resize(layout_.poly_num);
        std::iota(indices.begin(), indices.end(), 0);
        std::sort(indices.begin(), indices.end(), [&](size_t i, size_t j) {
            try {
                double area_i = safe_to_double(geo::pwh_area(*layout_.sheet_parts[0][i].base));
                double area_j = safe_to_double(geo::pwh_area(*layout_.sheet_parts[0][j].base));
                return area_i > area_j;
            }
            catch (...) {
                return false;
            }
        });
    }

    std::vector<size_t> placed_indices;
    
    // 放置第一个零件：稍微远离中心，给其他零件留出空间
    if (!indices.empty()) {
        size_t first_idx = indices[0];
        double radius = diameter / 2.0;
        auto& shape = layout_.sheet_parts[0][first_idx];
        
        // 先更新一次以确保transformed是最新的
        shape.update();
        auto bbox = shape.transformed.bbox();
        double shape_width = safe_to_double(bbox.xmax() - bbox.xmin());
        double shape_height = safe_to_double(bbox.ymax() - bbox.ymin());
        
        // 稍微远离中心：使用半径的30%作为偏移，给其他零件留出空间
        double offset_ratio = 0.3;
        double offset_x = radius * offset_ratio;
        double offset_y = radius * offset_ratio;
        
        shape.set_translate(geo::FT(radius - shape_width / 2.0 + offset_x), 
                          geo::FT(radius - shape_height / 2.0 + offset_y));
        shape.update();
        
        // 严格验证：确保第一个零件也在圆内且无重叠（虽然此时没有其他零件）
        std::vector<size_t> empty_placed; // 空列表，因为这是第一个零件
        if (!is_valid_placement(first_idx, empty_placed)) {
            // 如果中心位置无效，尝试使用place_shape_with_nfp寻找有效位置
            if (!place_shape_with_nfp(first_idx, empty_placed, params_.try_all_rotations)) {
                return false; // 无法放置第一个零件
            }
        }
        
        placed_indices.push_back(first_idx);
    }

    // 依次放置剩余零件
    for (size_t i = 1; i < indices.size(); ++i) {
        if (requestQuit && *requestQuit) {
            return false;
        }

        size_t shape_idx = indices[i];
        bool placed = false;

        // 优化策略：即使允许旋转，也优先尝试“保持当前朝向”放置；
        // 只有在固定朝向无法放置时，才启用多角度旋转搜索。
        if (params_.try_all_rotations) {
            // 1）先不旋转（只用当前rotation）尝试放置
            placed = place_shape_with_nfp(shape_idx, placed_indices, /*try_all_rotations=*/false);
            // 2）如果失败，再启用多角度旋转搜索
            if (!placed) {
                placed = place_shape_with_nfp(shape_idx, placed_indices, /*try_all_rotations=*/true);
            }
        } else {
            // 完全不旋转的模式，保持原有行为
            placed = place_shape_with_nfp(shape_idx, placed_indices, /*try_all_rotations=*/false);
        }

        if (placed) {
            // 成功放置，添加到已放置列表
            placed_indices.push_back(shape_idx);
        }
        // 如果无法放置，继续尝试下一个零件（不直接返回false）
        // 这样可以让算法尝试放置尽可能多的零件，即使不是所有零件都能放置
        
        // 每放置5个零件，报告一次进度
        if (progress_callback && placed_indices.size() % 5 == 0) {
            // 计算当前利用率并更新最佳结果
            double current_util = calculate_utilization();
            if (current_util > best_utilization_) {
                best_utilization_ = current_util;
                layout_.best_utilization = best_utilization_;
                layout_.best_result = layout_.sheet_parts;
            }
            (*progress_callback)(layout_);
        }
    }
    
    // 放置完成后，将未放置的零件移动到板材外面（避免影响可视化）
    double radius_final = diameter / 2.0;
    double outside_offset_final = -radius_final * 2.5; // 放在板材外足够远的位置
    std::set<size_t> placed_set(placed_indices.begin(), placed_indices.end());
    for (size_t i = 0; i < layout_.poly_num; ++i) {
        if (placed_set.find(i) == placed_set.end()) {
            // 未放置的零件，移动到板材外面
            auto& shape = layout_.sheet_parts[0][i];
            // 使用不同的偏移，避免重叠显示
            double offset_x = outside_offset_final + i * radius_final * 0.1;
            double offset_y = outside_offset_final;
            shape.set_translate(geo::FT(offset_x), geo::FT(offset_y));
            shape.update();
        }
    }
    
    // 放置完成后，更新最佳结果
    double final_util = calculate_utilization();
    if (final_util > best_utilization_) {
        best_utilization_ = final_util;
        layout_.best_utilization = best_utilization_;
        layout_.best_result = layout_.sheet_parts;
        
        if (progress_callback) {
            (*progress_callback)(layout_);
        }
    }

    // 如果成功放置了至少一个零件，返回true
    // 这样即使不是所有零件都能放置，也能继续优化已放置的零件
    return !placed_indices.empty();
}

void CircleNesting::compact_layout(size_t iterations, volatile bool* requestQuit,
                                    std::function<void(const Layout&)>* progress_callback) {
    if (layout_.sheet_parts.empty() || layout_.sheet_parts[0].empty()) {
        return;
    }

    // 缓存常用值，避免重复计算
    const double radius = safe_to_double(layout_.sheets[0].diameter) / 2.0;
    const double center_x = radius;
    const double center_y = radius;
    const double step_size = radius * params_.compact_step_size;
    
    // 模拟退火参数
    double temperature = params_.sa_initial_temp;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(0.0, 1.0);

    double current_utilization = calculate_utilization();
    double best_utilization_this_compact = current_utilization;

    // 预分配，循环内复用
    std::vector<size_t> indices(layout_.poly_num);
    std::iota(indices.begin(), indices.end(), 0);
    
    std::vector<size_t> other_indices;
    other_indices.reserve(layout_.poly_num);
    
    std::vector<std::pair<double, double>> directions;
    directions.reserve(4);

    for (size_t iter = 0; iter < iterations; ++iter) {
        if (requestQuit && *requestQuit) {
            break;
        }

        bool improved = false;
        
        // 随机打乱顺序
        std::shuffle(indices.begin(), indices.end(), gen);

        for (size_t idx : indices) {
            auto& shape = layout_.sheet_parts[0][idx];
            
            // ============================================================
            // 额外一步：使用 NFP 进行局部“贴合重定位”（图形匹配式局部优化）
            // 偶尔将当前零件从当前位置拿起，重新通过 NFP 候选点寻找更紧凑的位置，
            // 相当于对该零件做一次“局部重新排布”。
            // ============================================================
            if (params_.prioritize_compact_positions && iter % 20 == 0) {
                // 构造"其他零件"集合（复用预分配的容器）
                other_indices.clear();
                for (size_t j = 0; j < layout_.poly_num; ++j) {
                    if (j != idx) other_indices.push_back(j);
                }

                auto old_x = shape.get_translate_ft_x();
                auto old_y = shape.get_translate_ft_y();
                uint32_t old_rot = shape.get_rotation();

                // 使用现有 NFP 放置逻辑，对该零件做一次局部重定位
                bool placed_better = place_shape_with_nfp(idx, other_indices, params_.try_all_rotations);
                if (placed_better && is_valid_placement(idx, other_indices)) {
                    double new_util = calculate_utilization();
                    double delta = new_util - current_utilization;
                    bool accept = false;
                    if (delta > 0) {
                        accept = true;
                        improved = true;
                        current_utilization = new_util;
                        if (new_util > best_utilization_this_compact) {
                            best_utilization_this_compact = new_util;
                        }
                    } else if (params_.use_simulated_annealing && temperature > 0) {
                        double prob = std::exp(delta / temperature);
                        if (dis(gen) < prob) {
                            accept = true;
                        }
                    }

                    if (!accept) {
                        shape.set_rotation(old_rot);
                        shape.set_translate(old_x, old_y);
                        shape.update();
                    }
                } else {
                    shape.set_rotation(old_rot);
                    shape.set_translate(old_x, old_y);
                    shape.update();
                }
            }

            // 多方向搜索：向中心、向最近零件、随机方向（复用预分配的容器）
            directions.clear();
            
            // 方向1：向中心
            auto bbox = shape.transformed.bbox();
            double cx = safe_to_double(bbox.xmin() + bbox.xmax()) / 2.0;
            double cy = safe_to_double(bbox.ymin() + bbox.ymax()) / 2.0;
            double dx_center = center_x - cx;
            double dy_center = center_y - cy;
            double dist_center = std::sqrt(dx_center * dx_center + dy_center * dy_center);
            if (dist_center > 1e-6) {
                directions.push_back({dx_center / dist_center, dy_center / dist_center});
            }
            
            // 方向2：向最近已放置零件（填充空隙）
            double min_dist = std::numeric_limits<double>::max();
            double dx_to_part = 0, dy_to_part = 0;
            for (size_t j = 0; j < layout_.poly_num; ++j) {
                if (j == idx) continue;
                auto bbox_j = layout_.sheet_parts[0][j].transformed.bbox();
                double cx_j = safe_to_double(bbox_j.xmin() + bbox_j.xmax()) / 2.0;
                double cy_j = safe_to_double(bbox_j.ymin() + bbox_j.ymax()) / 2.0;
                double dist = std::sqrt((cx - cx_j) * (cx - cx_j) + (cy - cy_j) * (cy - cy_j));
                if (dist < min_dist && dist > 1e-6) {
                    min_dist = dist;
                    dx_to_part = cx_j - cx;
                    dy_to_part = cy_j - cy;
                    double norm = std::sqrt(dx_to_part * dx_to_part + dy_to_part * dy_to_part);
                    if (norm > 1e-6) {
                        dx_to_part /= norm;
                        dy_to_part /= norm;
                    }
                }
            }
            if (min_dist < std::numeric_limits<double>::max()) {
                directions.push_back({dx_to_part, dy_to_part});
            }
            
            // 方向3：随机方向（探索性搜索）
            double random_angle = dis(gen) * 2.0 * Sheet::kPi;
            directions.push_back({std::cos(random_angle), std::sin(random_angle)});

            // 尝试每个方向
            for (const auto& dir : directions) {
                double current_x = shape.get_translate_double_x();
                double current_y = shape.get_translate_double_y();
                
                // 尝试移动（可能接受次优解，模拟退火）
                double move_x = dir.first * step_size;
                double move_y = dir.second * step_size;
                double new_x = current_x + move_x;
                double new_y = current_y + move_y;

                auto old_x = shape.get_translate_ft_x();
                auto old_y = shape.get_translate_ft_y();
                
                shape.set_translate(geo::FT(new_x), geo::FT(new_y));
                shape.update();

                // 严格检查：确保移动后仍然遵守两个约束（刀头不能超出板材，刀头之间不能重叠）
                std::vector<size_t> other_indices;
                for (size_t j = 0; j < layout_.poly_num; ++j) {
                    if (j != idx) {
                        other_indices.push_back(j);
                    }
                }

                if (is_valid_placement(idx, other_indices)) {
                    double new_utilization = calculate_utilization();
                    double delta = new_utilization - current_utilization;
                    
                    // 接受改进或按概率接受次优解（模拟退火）
                    bool accept = false;
                    if (delta > 0) {
                        accept = true;
                        improved = true;
                        current_utilization = new_utilization;
                        if (new_utilization > best_utilization_this_compact) {
                            best_utilization_this_compact = new_utilization;
                        }
                    } else if (params_.use_simulated_annealing && temperature > 0) {
                        double prob = std::exp(delta / temperature);
                        if (dis(gen) < prob) {
                            accept = true;
                        }
                    }
                    
                    if (!accept) {
                        // 恢复
                        shape.set_translate(old_x, old_y);
                        shape.update();
                    }
                } else {
                    // 恢复
                    shape.set_translate(old_x, old_y);
                    shape.update();
                }
            }
        }

        // 降低温度（模拟退火）
        if (params_.use_simulated_annealing) {
            temperature *= params_.sa_cooling_rate;
        }

        // 如果改进了利用率，更新最佳结果
        if (improved && best_utilization_this_compact > best_utilization_) {
            best_utilization_ = best_utilization_this_compact;
            layout_.best_utilization = best_utilization_;
            layout_.best_result = layout_.sheet_parts;
        }

        // 每10次迭代报告一次进度（只在有改进时更新best_result，确保显示的是最佳值）
        if (progress_callback && iter % 10 == 0) {
            // 确保 best_result 是最新的最佳状态
            if (best_utilization_this_compact > best_utilization_) {
                best_utilization_ = best_utilization_this_compact;
                layout_.best_utilization = best_utilization_;
                layout_.best_result = layout_.sheet_parts;
            }
            (*progress_callback)(layout_);
        }

        if (!improved && iter > 50 && !params_.use_simulated_annealing) {
            // 如果没有改进且已经迭代多次，提前退出（模拟退火模式下继续）
            break;
        }
    }
    
    // 紧凑化结束后，确保更新最佳结果
    double final_utilization = calculate_utilization();
    if (final_utilization > best_utilization_) {
        best_utilization_ = final_utilization;
        layout_.best_utilization = best_utilization_;
        layout_.best_result = layout_.sheet_parts;
        
        if (progress_callback) {
            (*progress_callback)(layout_);
        }
    }
}

bool CircleNesting::optimize_diameter(double target_utilization, volatile bool* requestQuit,
                                     std::function<void(const Layout&)> progress_callback) {
    double theoretical_min = calculate_theoretical_min_diameter();
    double current_diameter = safe_to_double(layout_.sheets[0].diameter);
    
    double best_diameter = current_diameter;
    double best_util = 0.0;
    double min_diameter = theoretical_min * params_.min_diameter_ratio;
    
    if (params_.use_binary_search) {
        // 使用二分查找优化直径
        best_diameter = binary_search_diameter(min_diameter, current_diameter, 
                                              target_utilization, requestQuit, progress_callback);
        best_util = layout_.best_utilization;
    } else {
        // 线性搜索（保留作为备选）
        double step = current_diameter * params_.diameter_step_ratio;
        
        for (size_t iter = 0; iter < params_.diameter_optimization_iterations; ++iter) {
            if (requestQuit && *requestQuit) {
                break;
            }

            double test_diameter = current_diameter - step * iter;
            if (test_diameter < min_diameter) {
                break;
            }

            if (try_place_all_parts(test_diameter, requestQuit, &progress_callback)) {
                compact_layout(params_.compact_iterations, requestQuit, &progress_callback);
                
                double util = calculate_utilization();
                
                if (util > best_util) {
                    best_util = util;
                    best_diameter = test_diameter;
                    
                    layout_.best_result = layout_.sheet_parts;
                    layout_.best_utilization = util;
                    
                    if (progress_callback) {
                        progress_callback(layout_);
                    }
                }
                
                if (util >= target_utilization) {
                    update_circle_diameter(best_diameter);
                    return true;
                }
            }
        }
    }

    // 使用最佳直径，进行最终优化
    if (best_util > 0) {
        update_circle_diameter(best_diameter);
        if (!try_place_all_parts(best_diameter, requestQuit, &progress_callback)) {
            return false;
        }
        // 最终紧凑化（使用更多迭代）
        compact_layout(params_.compact_iterations, requestQuit, &progress_callback);
        best_utilization_ = calculate_utilization();
        layout_.best_utilization = best_utilization_;
        layout_.best_result = layout_.sheet_parts;
        
        if (progress_callback) {
            progress_callback(layout_);
        }
        
        return best_utilization_ >= target_utilization;
    }

    return false;
}

bool CircleNesting::optimize_array_mode(double target_utilization, volatile bool* requestQuit,
                                        std::function<void(const Layout&)> progress_callback) {
    // 阵列模式：在当前直径下，通过搜索一组候选步距(pitch)，
    // 近似寻找“放件数尽量多、利用率尽量高”的周期阵列（简化版最小阵列思想）。
    if (layout_.sheet_parts.empty() || layout_.sheet_parts[0].empty()) {
        return false;
    }

    double radius = safe_to_double(layout_.sheets[0].diameter) / 2.0;
    double center_x = radius;
    double center_y = radius;

    // 选第一个零件作为阵列单元的尺寸参考（假定大部分零件尺寸相近）
    const auto& first_shape = layout_.sheet_parts[0][0];
    geo::Polygon_with_holes_2 base_poly = *first_shape.base;
    auto bbox = base_poly.outer_boundary().bbox();
    double shape_width = safe_to_double(bbox.xmax() - bbox.xmin());
    double shape_height = safe_to_double(bbox.ymax() - bbox.ymin());
    if (shape_width <= 0 || shape_height <= 0) {
        return false;
    }

    // 基准阵列步距：用户指定优先，否则自动 = 尺寸 + 适当间隙（取10%尺寸）
    double gap_x = 0.1 * shape_width;
    double gap_y = 0.1 * shape_height;
    double margin = std::max(0.0, params_.array_margin);

    // 构造一组候选步距缩放因子（围绕基准pitch做搜索）
    std::vector<double> scale_candidates = { 0.9, 1.0, 1.1 };
    double base_pitch_x = (params_.array_pitch_x > 0.0) ? params_.array_pitch_x : (shape_width + gap_x);
    double base_pitch_y = (params_.array_pitch_y > 0.0) ? params_.array_pitch_y : (shape_height + gap_y);
    if (base_pitch_x <= 0 || base_pitch_y <= 0) {
        return false;
    }

    // 备份初始布局（仅关心 sheet_parts）
    auto original_parts = layout_.sheet_parts;

    double best_util = 0.0;
    size_t best_placed_count = 0;
    std::vector<std::vector<TransformedShape>> best_parts;

    for (double sx : scale_candidates) {
        for (double sy : scale_candidates) {
            if (requestQuit && *requestQuit) {
                break;
            }

            double pitch_x = base_pitch_x * sx;
            double pitch_y = base_pitch_y * sy;
            if (pitch_x <= 0 || pitch_y <= 0) {
                continue;
            }

            // 还原布局
            layout_.sheet_parts = original_parts;

            // 先把所有零件移到板外，避免视觉干扰
            double outside_offset = -radius * 2.0;
            for (auto& shape : layout_.sheet_parts[0]) {
                shape.set_translate(geo::FT(outside_offset), geo::FT(outside_offset));
                shape.update();
            }

            // 为当前 pitch 生成阵列格点
            std::vector<geo::Point_2> grid_points;
            int max_ix = static_cast<int>(std::ceil((radius - margin) / pitch_x)) + 2;
            int max_iy = static_cast<int>(std::ceil((radius - margin) / pitch_y)) + 2;

            for (int iy = -max_iy; iy <= max_iy; ++iy) {
                for (int ix = -max_ix; ix <= max_ix; ++ix) {
                    double gx = center_x + ix * pitch_x;
                    double gy = center_y + iy * pitch_y;

                    double tx = gx - shape_width / 2.0;
                    double ty = gy - shape_height / 2.0;

                    double corners_x[2] = { tx, tx + shape_width };
                    double corners_y[2] = { ty, ty + shape_height };
                    double max_dist_sq = 0.0;
                    for (int cx_i = 0; cx_i < 2; ++cx_i) {
                        for (int cy_i = 0; cy_i < 2; ++cy_i) {
                            double dx = corners_x[cx_i] - center_x;
                            double dy = corners_y[cy_i] - center_y;
                            double d2 = dx * dx + dy * dy;
                            if (d2 > max_dist_sq) {
                                max_dist_sq = d2;
                            }
                        }
                    }
                    double limit_r = std::max(0.0, radius - margin);
                    if (max_dist_sq <= limit_r * limit_r) {
                        grid_points.emplace_back(geo::FT(tx), geo::FT(ty));
                    }
                }
            }

            if (grid_points.empty()) {
                continue;
            }

            // 按“离圆心距离”排序，优先内部格点
            std::sort(grid_points.begin(), grid_points.end(),
                [&](const geo::Point_2& a, const geo::Point_2& b) {
                    double ax = safe_to_double(a.x()) + shape_width / 2.0;
                    double ay = safe_to_double(a.y()) + shape_height / 2.0;
                    double bx = safe_to_double(b.x()) + shape_width / 2.0;
                    double by = safe_to_double(b.y()) + shape_height / 2.0;
                    double da2 = (ax - center_x) * (ax - center_x) + (ay - center_y) * (ay - center_y);
                    double db2 = (bx - center_x) * (bx - center_x) + (by - center_y) * (by - center_y);
                    return da2 < db2;
                });

            std::vector<size_t> placed_indices;
            size_t grid_idx = 0;

            // 固定旋转：阵列模式一般不希望乱转，这里用当前rotation，不额外旋转
            for (size_t i = 0; i < layout_.poly_num && grid_idx < grid_points.size(); ++i) {
                if (requestQuit && *requestQuit) {
                    break;
                }

                auto& shape = layout_.sheet_parts[0][i];
                shape.update();

                for (; grid_idx < grid_points.size(); ++grid_idx) {
                    auto tx = grid_points[grid_idx].x();
                    auto ty = grid_points[grid_idx].y();

                    auto old_x = shape.get_translate_ft_x();
                    auto old_y = shape.get_translate_ft_y();

                    shape.set_translate(tx, ty);
                    shape.update();

                    if (is_valid_placement(i, placed_indices)) {
                        placed_indices.push_back(i);
                        ++grid_idx;
                        break;
                    } else {
                        shape.set_translate(old_x, old_y);
                        shape.update();
                    }
                }
            }

            // 未放置零件移到板外，方便查看
            double outside_offset_final = -radius * 2.5;
            std::set<size_t> placed_set(placed_indices.begin(), placed_indices.end());
            for (size_t i = 0; i < layout_.poly_num; ++i) {
                if (placed_set.find(i) == placed_set.end()) {
                    auto& shape = layout_.sheet_parts[0][i];
                    double offset_x = outside_offset_final + i * radius * 0.1;
                    double offset_y = outside_offset_final;
                    shape.set_translate(geo::FT(offset_x), geo::FT(offset_y));
                    shape.update();
                }
            }

            // 评估当前阵列：先看放了多少件，再看利用率
            size_t placed_count = placed_indices.size();
            double util = calculate_utilization();

            if (placed_count > best_placed_count ||
                (placed_count == best_placed_count && util > best_util)) {
                best_placed_count = placed_count;
                best_util = util;
                best_parts = layout_.sheet_parts;
            }
        }
    }

    if (best_placed_count == 0 || best_parts.empty()) {
        // 没有找到任何可行阵列
        layout_.sheet_parts = original_parts;
        best_utilization_ = 0.0;
        layout_.best_utilization = 0.0;
        layout_.best_result = layout_.sheet_parts;
        if (progress_callback) {
            progress_callback(layout_);
        }
        return false;
    }

    // 使用搜索到的最佳阵列布局
    layout_.sheet_parts = best_parts;
    best_utilization_ = best_util;
    layout_.best_utilization = best_utilization_;
    layout_.best_result = layout_.sheet_parts;

    if (progress_callback) {
        progress_callback(layout_);
    }

    return best_utilization_ >= target_utilization;
}

bool CircleNesting::optimize(double target_utilization, volatile bool* requestQuit,
                             std::function<void(const Layout&)> progress_callback) {
    best_utilization_ = 0.0;
    
    try {
        // 阵列模式：直接走阵列排样逻辑（不做直径搜索）
        if (params_.use_array_mode) {
            return optimize_array_mode(target_utilization, requestQuit, progress_callback);
        }

        // 首先尝试在当前直径下放置
        if (!try_place_all_parts(safe_to_double(layout_.sheets[0].diameter), requestQuit, &progress_callback)) {
            return false;
        }

        // 紧凑化
        compact_layout(params_.compact_iterations, requestQuit, &progress_callback);
        
        // 确保 best_result 是最新的最佳状态（即使被停止）
        double current_util = calculate_utilization();
        if (current_util > best_utilization_) {
            best_utilization_ = current_util;
            layout_.best_utilization = best_utilization_;
            layout_.best_result = layout_.sheet_parts;
        }
        
        if (progress_callback) {
            progress_callback(layout_);
        }

        // 如果已达到目标，直接返回
        if (best_utilization_ >= target_utilization) {
            return true;
        }

        // 优化直径
        bool diameter_success = optimize_diameter(target_utilization, requestQuit, progress_callback);
        
        // 停止时，确保 best_result 是最新的（可能在直径优化过程中有改进）
        if (requestQuit && *requestQuit) {
            double final_util = calculate_utilization();
            if (final_util > best_utilization_) {
                best_utilization_ = final_util;
                layout_.best_utilization = best_utilization_;
                layout_.best_result = layout_.sheet_parts;
            }
        }
        
        return diameter_success;
    }
    catch (...) {
        return false;
    }
}

double CircleNesting::get_current_diameter() const {
    return current_diameter_;
}

double CircleNesting::calculate_shape_complexity(size_t shape_idx) const {
    if (shape_idx >= layout_.sheet_parts[0].size()) {
        return 0.0;
    }
    
    try {
        const auto& shape = layout_.sheet_parts[0][shape_idx];
        const auto& poly = *shape.base;
        
        // 计算面积
        double area = safe_to_double(geo::pwh_area(poly));
        if (area <= 0) return 0.0;
        
        // 计算周长
        double perimeter = 0.0;
        for (auto e = poly.outer_boundary().edges_begin(); 
             e != poly.outer_boundary().edges_end(); ++e) {
            double dx = safe_to_double(e->target().x() - e->source().x());
            double dy = safe_to_double(e->target().y() - e->source().y());
            perimeter += std::sqrt(dx * dx + dy * dy);
        }
        
        // 计算长宽比
        auto bbox = poly.bbox();
        double width = safe_to_double(bbox.xmax() - bbox.xmin());
        double height = safe_to_double(bbox.ymax() - bbox.ymin());
        double aspect_ratio = (width > height) ? width / height : height / width;
        
        // 计算顶点数（复杂度指标）
        size_t vertex_count = poly.outer_boundary().size();
        
        // 综合评分：面积大、周长/面积比大（复杂形状）、长宽比大（细长形状）优先
        // 这些形状通常更难放置，应该优先处理
        double complexity = area * (1.0 + perimeter / area * 0.1) * (1.0 + aspect_ratio * 0.1) * (1.0 + vertex_count * 0.01);
        
        return complexity;
    }
    catch (...) {
        return 0.0;
    }
}

std::vector<size_t> CircleNesting::smart_order_parts() const {
    std::vector<size_t> indices(layout_.poly_num);
    std::iota(indices.begin(), indices.end(), 0);
    
    // 按综合评分排序：复杂度高的优先
    std::sort(indices.begin(), indices.end(), [&](size_t i, size_t j) {
        try {
            double complexity_i = calculate_shape_complexity(i);
            double complexity_j = calculate_shape_complexity(j);
            return complexity_i > complexity_j;
        }
        catch (...) {
            return false;
        }
    });
    
    return indices;
}

// ============================================================================
// 调度算法：尝试多种放置顺序，返回能放置最多零件且利用率最高的顺序
// ============================================================================
std::vector<size_t> CircleNesting::schedule_best_order(double diameter, volatile bool* requestQuit,
                                                        std::function<void(const Layout&)>* progress_callback) {
    if (!params_.use_scheduling || params_.scheduling_attempts <= 1) {
        return smart_order_parts();
    }

    std::random_device rd;
    std::mt19937 gen(rd());

    std::vector<size_t> best_order = smart_order_parts();
    size_t best_placed_count = 0;
    double best_util = 0.0;
    
    // 内存优化：只在找到更好结果时才保存布局
    bool has_best_layout = false;
    std::vector<std::vector<TransformedShape>> best_layout;

    // 内存优化：使用 move 语义备份，减少拷贝
    auto original_parts = std::move(layout_.sheet_parts);
    layout_.sheet_parts = original_parts; // 恢复一份用于操作

    // 内存优化：预分配 candidate_order
    std::vector<size_t> candidate_order;
    candidate_order.reserve(layout_.poly_num);
    
    // 内存优化：预分配 placed_indices
    std::vector<size_t> placed_indices;
    placed_indices.reserve(layout_.poly_num);
    
    // 内存优化：复用空列表
    const std::vector<size_t> empty_placed;

    for (size_t attempt = 0; attempt < params_.scheduling_attempts; ++attempt) {
        if (requestQuit && *requestQuit) break;

        // 生成候选顺序（复用 candidate_order）
        candidate_order.clear();
        if (attempt == 0) {
            candidate_order = smart_order_parts();
        } else if (attempt == 1) {
            candidate_order.resize(layout_.poly_num);
            std::iota(candidate_order.begin(), candidate_order.end(), 0);
            std::sort(candidate_order.begin(), candidate_order.end(), [&](size_t i, size_t j) {
                try {
                    double area_i = safe_to_double(geo::pwh_area(*original_parts[0][i].base));
                    double area_j = safe_to_double(geo::pwh_area(*original_parts[0][j].base));
                    return area_i > area_j;
                } catch (...) {
                    return false;
                }
            });
        } else if (params_.use_random_shuffle) {
            candidate_order = smart_order_parts();
            std::shuffle(candidate_order.begin(), candidate_order.end(), gen);
        } else {
            continue;
        }

        // 恢复初始状态
        layout_.sheet_parts = original_parts;
        update_circle_diameter(diameter);

        // 复用 placed_indices
        placed_indices.clear();
        double radius = diameter / 2.0;

        // 放置第一个零件
        if (!candidate_order.empty()) {
            size_t first_idx = candidate_order[0];
            auto& shape = layout_.sheet_parts[0][first_idx];
            shape.update();
            auto bbox = shape.transformed.bbox();
            double shape_width = safe_to_double(bbox.xmax() - bbox.xmin());
            double shape_height = safe_to_double(bbox.ymax() - bbox.ymin());
            shape.set_translate(geo::FT(radius - shape_width / 2.0 + radius * 0.3),
                                geo::FT(radius - shape_height / 2.0 + radius * 0.3));
            shape.update();

            if (is_valid_placement(first_idx, empty_placed)) {
                placed_indices.push_back(first_idx);
            } else if (place_shape_with_nfp(first_idx, empty_placed, params_.try_all_rotations)) {
                placed_indices.push_back(first_idx);
            }
        }

        // 放置剩余零件
        for (size_t i = 1; i < candidate_order.size(); ++i) {
            if (requestQuit && *requestQuit) break;
            size_t shape_idx = candidate_order[i];
            bool placed = place_shape_with_nfp(shape_idx, placed_indices, false);
            if (!placed && params_.try_all_rotations) {
                placed = place_shape_with_nfp(shape_idx, placed_indices, true);
            }
            if (placed) {
                placed_indices.push_back(shape_idx);
            }
        }

        // 评估
        size_t current_placed = placed_indices.size();
        if (current_placed > best_placed_count) {
            best_placed_count = current_placed;
            best_util = calculate_utilization();
            best_order = candidate_order;
            best_layout = layout_.sheet_parts;
            has_best_layout = true;
        } else if (current_placed == best_placed_count) {
            double current_util = calculate_utilization();
            if (current_util > best_util) {
                best_util = current_util;
                best_order = candidate_order;
                best_layout = layout_.sheet_parts;
                has_best_layout = true;
            }
        }
    }

    // 恢复最佳布局
    if (has_best_layout) {
        layout_.sheet_parts = std::move(best_layout);
    } else {
        layout_.sheet_parts = std::move(original_parts);
    }

    return best_order;
}

double CircleNesting::binary_search_diameter(double min_diameter, double max_diameter,
                                             double target_utilization, volatile bool* requestQuit,
                                             std::function<void(const Layout&)> progress_callback) {
    double best_diameter = max_diameter;
    double best_utilization = 0.0;
    const double tolerance = 0.001; // 直径容差
    const size_t max_iterations = 50;
    
    for (size_t iter = 0; iter < max_iterations; ++iter) {
        if (requestQuit && *requestQuit) {
            break;
        }
        
        if (max_diameter - min_diameter < tolerance) {
            break;
        }
        
        double test_diameter = (min_diameter + max_diameter) / 2.0;
        
        // 使用调度算法尝试多种顺序，找到最佳放置方案
        schedule_best_order(test_diameter, requestQuit, &progress_callback);
        
        // 检查是否成功放置了零件
        size_t placed_count = 0;
        for (const auto& shape : layout_.sheet_parts[0]) {
            auto bbox = shape.transformed.bbox();
            double cx = safe_to_double(bbox.xmin() + bbox.xmax()) / 2.0;
            double cy = safe_to_double(bbox.ymin() + bbox.ymax()) / 2.0;
            double radius = test_diameter / 2.0;
            if (cx > 0 && cy > 0 && cx < test_diameter && cy < test_diameter) {
                placed_count++;
            }
        }
        
        if (placed_count > 0) {
            // 紧凑化
            compact_layout(params_.compact_iterations / 2, requestQuit, &progress_callback);
            
            double util = calculate_utilization();
            
            if (util > best_utilization) {
                best_utilization = util;
                best_diameter = test_diameter;
                
                layout_.best_result = layout_.sheet_parts;
                layout_.best_utilization = util;
                
                if (progress_callback) {
                    progress_callback(layout_);
                }
            }
            
            if (util >= target_utilization) {
                // 可以尝试更小的直径
                max_diameter = test_diameter;
            } else {
                // 需要更大的直径
                min_diameter = test_diameter;
            }
        } else {
            // 无法放置，需要更大的直径
            min_diameter = test_diameter;
        }
    }
    
    return best_diameter;
}

} // namespace nesting

