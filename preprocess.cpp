#include "preprocess.h"

namespace nesting {
    namespace geo {
        void preprocess(std::vector<nesting::Item>& items,
            double offset,
            bool need_simplify) {
            for (auto& item : items) {
                auto& p = item.poly;
                auto& outer = p.outer_boundary();
                // 外轮廓需要是逆时针方向
                if (outer.is_clockwise_oriented()) {
                    outer.reverse_orientation();
                }
                for (auto& h : p.holes()) {
                    // 孔洞需要是顺时针方向
                    if (h.is_counterclockwise_oriented()) {
                        h.reverse_orientation();
                    }
                }
                // TODO 设置并捕获异常
                if (!is_valid_polygon_with_holes(p)) {
                    throw std::runtime_error("Input polygon is not valid");
                }
                // 简化多边形
                if (need_simplify) {
                    item.simplified = simplify(p);
                }
            }
        }

        void preprocess(Sheet& sheet,
            double top_offset,
            double left_offset,
            double bottom_offset,
            double right_offset) {
            // 对矩形与圆形分别处理：圆形用height==0作为标志
            if (sheet.height == FT(0)) {
                // 圆板：根据四边偏置缩减直径，并同步更新实际圆形多边形
                // 每边偏置对直径贡献一次，合计对直径的缩减为四边之和
                double diameter_delta = -(top_offset + bottom_offset + left_offset + right_offset);
                // 计算新直径：原始直径存储在 sheet.diameter
                double old_d = CGAL::to_double(sheet.diameter);
                double new_d = old_d + diameter_delta;
                if (new_d < 0.0) new_d = 0.0;

                // 更新直径与宽度（用于 UI 显示）；保持 height==0 作为圆形标志
                sheet.diameter = FT(new_d);
                sheet.set_width(FT(new_d));
                // 不修改height，保持为0

                // 重新构建圆形板材的多边形（以正多边形近似），确保 IFR 和放置边界正确
                // 使用 sheet.segments 来决定分段数
                size_t segs = sheet.segments;
                const double kPi = Sheet::kPi;
                geo::Polygon_2 boundary;
                boundary.clear();
                // 圆心在 (new_d/2, new_d/2)，半径 new_d/2
                double cx = new_d / 2.0;
                double cy = new_d / 2.0;
                double r = new_d / 2.0;
                if (segs < 16) segs = 16;
                for (size_t i = 0; i < segs; ++i) {
                    double theta = (2.0 * kPi * static_cast<double>(i)) / static_cast<double>(segs);
                    double x = cx + r * std::cos(theta);
                    double y = cy + r * std::sin(theta);
                    boundary.push_back(geo::Point_2(FT(x), FT(y)));
                }
                // 保证外轮廓为逆时针
                if (boundary.is_clockwise_oriented()) {
                    boundary.reverse_orientation();
                }
                sheet.sheet = geo::Polygon_with_holes_2(boundary);
                return;
            }
            // 矩形板材：按四边偏置缩减长宽，并同步更新
            double vertical = 0;
            double horizontal = 0;
            vertical -= top_offset;
            vertical -= bottom_offset;
            horizontal -= left_offset;
            horizontal -= right_offset;
            sheet.set_height(sheet.height + vertical);
            sheet.set_width(sheet.width + horizontal);
        }
    }  // namespace geo
}  // namespace nesting