#pragma once
#include "basic.h"
#include <cmath>
#include <CGAL/number_utils.h>
namespace nesting {
    struct Sheet {
        enum class ShapeType { Rectangle, Circle };
        static constexpr double kPi = 3.14159265358979323846;
        geo::FT width;
        geo::FT height;
        geo::FT diameter{ 0 };
        size_t segments{ 128 }; // number of segments used to approximate circle
        ShapeType type{ ShapeType::Rectangle };
        geo::Polygon_with_holes_2 sheet;
        // 构造函数：矩形
        explicit Sheet(geo::FT _width, geo::FT _height)
            : width(_width), height(_height), type(ShapeType::Rectangle) {
            auto& outer = sheet.outer_boundary();
            outer.push_back(geo::Point_2(0, 0));
            outer.push_back(geo::Point_2(_width, 0));
            outer.push_back(geo::Point_2(_width, _height));
            outer.push_back(geo::Point_2(0, _height));
        }
        // 工厂方法：圆形（用正多边形近似），diameter为直径
        static Sheet make_circle(geo::FT d, size_t segments = 128) {
            Sheet s(d, d);
            s.type = ShapeType::Circle;
            s.diameter = d;
            s.segments = segments;
            auto& outer = s.sheet.outer_boundary();
            outer.clear();
            double r = CGAL::to_double(d) / 2.0;
            double cx = CGAL::to_double(d) / 2.0;
            double cy = CGAL::to_double(d) / 2.0;
            if (s.segments < 16) s.segments = 16; // 避免过少的段数
            for (size_t i = 0; i < s.segments; ++i) {
                double theta = (2.0 * kPi * static_cast<double>(i)) / static_cast<double>(s.segments);
                double x = cx + r * std::cos(theta);
                double y = cy + r * std::sin(theta);
                outer.push_back(geo::Point_2(geo::FT(x), geo::FT(y)));
            }
            return s;
        }
        // 将sheet的宽度变为_width（仅矩形有效）
        inline void set_width(geo::FT _width) {
            if (type != ShapeType::Rectangle) { return; }
            width = _width;
            auto& outer = sheet.outer_boundary();
            auto p1 = outer.vertices_begin() + 1;
            outer.erase(p1);
            outer.erase(p1);
            outer.insert(p1, geo::Point_2(_width, height));
            outer.insert(p1, geo::Point_2(_width, 0));
        }
        // 将sheet的高度变为_height（仅矩形有效）
        inline void set_height(geo::FT _height) {
            if (type != ShapeType::Rectangle) { return; }
            height = _height;
            auto& outer = sheet.outer_boundary();
            auto p1 = outer.vertices_begin() + 2;
            outer.erase(p1);
            outer.erase(p1);
            outer.insert(p1, geo::Point_2(0, _height));
            outer.insert(p1, geo::Point_2(width, _height));
        }
    };
}  // namespace nesting