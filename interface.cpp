#include "interface.h"

namespace nesting {

    void start(const size_t max_time,
        Layout& layout,
        std::function<void(const Solution&)> ProgressHandler,
        volatile bool* requestQuit) {
        nesting::GOMH(layout, max_time, ProgressHandler, requestQuit);
    }
    Preprocess preprocess(
        const bool need_simplify,
        const double top_offset,
        const double left_offset,
        const double bottom_offset,
        const double right_offset,
        const double part_offset,
        const double sheet_width,
        const double sheet_height,
        const std::vector<nesting::geo::Polygon_with_holes_2>& polygons,
        const std::vector<uint32_t>& items_rotations,
        const std::vector<uint32_t>& items_quantity,
        const size_t circle_segments) {
        std::vector<nesting::Item> original_items;
        size_t size = polygons.size();
        for (size_t i = 0; i < size; i++) {
            original_items.emplace_back(items_quantity[i], polygons[i],
                items_rotations[i]);
        }
        FT area(0);
        for (auto& i : original_items) {
            area += geo::pwh_area(i.poly) * FT(i.quantity);
        }
        // 处理sheet：当sheet_height==0时，按圆板处理，sheet_width为直径
        std::vector<nesting::Sheet> original_sheets;
        if (sheet_height == 0) {
            original_sheets.push_back(nesting::Sheet::make_circle(FT(sheet_width), circle_segments));
        } else {
            original_sheets.emplace_back(FT(sheet_width), FT(sheet_height));
        }
        std::vector<nesting::Sheet> sheets;
        std::vector<nesting::Item> items;
        std::copy(original_items.begin(), original_items.end(),
            std::back_inserter(items));
        std::copy(original_sheets.begin(), original_sheets.end(),
            std::back_inserter(sheets));
        geo::preprocess(items, part_offset / 2, need_simplify);
        geo::preprocess(sheets[0], top_offset, left_offset, bottom_offset,
            right_offset);
        std::vector<nesting::Item> simplified_items;
        std::copy(items.begin(), items.end(), std::back_inserter(simplified_items));
        for (size_t i = 0; i < items.size(); i++) {
            auto& item = items[i];
            geo::offset_polygon(item.poly, part_offset / 2);
            auto first = item.poly.outer_boundary().vertices_begin();
            Transformation translate(CGAL::TRANSLATION,
                Vector_2(-first->x(), -first->y()));
            simplified_items[i].poly =
                geo::transform_polygon_with_holes(translate, simplified_items[i].poly);
        }
        nesting::Layout layout(items, sheets, area);
        return Preprocess(layout, original_items, simplified_items);
    }
}  // namespace nesting
