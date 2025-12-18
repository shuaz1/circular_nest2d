#include "genetic_algorithm.h"
#include "nesting.h"
#include <CGAL/boolean_set_operations_2.h>
#include <CGAL/Polygon_2.h>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <atomic>
#include <limits>

namespace nesting {

// Robust conversion from CGAL::FT to double using intervals.
// This avoids CGAL::Uncertain_conversion_exception that can be thrown by to_double.
static double safe_to_double(const geo::FT& v) {
    // CGAL::to_interval is noexcept for exact kernels and returns a bounding interval.
    auto I = CGAL::to_interval(v);
    double a = I.first;
    double b = I.second;
    if (!std::isfinite(a) || !std::isfinite(b)) {
        return 0.0;
    }
    // Use the midpoint as a stable representative value.
    return 0.5 * (a + b);
}

// Helper: compute polygon_with_holes area via geo::pwh_area (use safe_to_double)
static double polygon_area_double(const geo::Polygon_with_holes_2& poly) {
    return safe_to_double(geo::pwh_area(poly));
}

GeneticAlgorithm::GeneticAlgorithm(Layout& layout)
    : layout_(layout), rng_(), current_generation_(0) {
    // Initialize best fitness history
    for (size_t i = 0; i < 100; i++) {
        best_fitness_history_[i] = 0.0;
    }
    // Initialize best solution with empty shapes
    best_solution_ = Chromosome();
}

void GeneticAlgorithm::set_parameters(const Parameters& params) {
    params_ = params;
}

std::vector<Chromosome> GeneticAlgorithm::initialize_population() {
    std::vector<Chromosome> population;

    // Check if we have any initial shapes
    std::vector<TransformedShape> initial_shapes;
    try {
    for (const auto& sheet_part : layout_.sheet_parts) {
        for (const auto& shape : sheet_part) {
            initial_shapes.push_back(shape);
        }
        }
    }
    catch (...) {
        std::clog << "GA: Failed to collect initial shapes" << std::endl;
        return population;
    }

    // If no initial shapes, return empty population
    if (initial_shapes.empty()) {
        std::clog << "GA: No initial shapes found" << std::endl;
        return population;
    }

    // Check if sheet is circular (needed for initialization strategy)
    const Sheet& sheet = layout_.sheets[0];
    bool is_circular = (sheet.type == Sheet::ShapeType::Circle);

    // First chromosome: for circular sheets, redistribute using angle-based strategy
    // For rectangular sheets, use initial layout as-is
    if (!initial_shapes.empty()) {
        Chromosome initial_chromosome;
        if (is_circular) {
            // Redistribute initial shapes using circular distribution
            std::vector<TransformedShape> redistributed_shapes;
            for (size_t idx = 0; idx < initial_shapes.size(); ++idx) {
                try {
                    TransformedShape redistributed = create_circular_distributed_shape(
                        initial_shapes[idx], idx, initial_shapes.size());
                    redistributed_shapes.push_back(redistributed);
                }
                catch (...) {
                    // If redistribution fails, use original shape
                    redistributed_shapes.push_back(initial_shapes[idx]);
                }
            }
            initial_chromosome = Chromosome(redistributed_shapes);
        } else {
            initial_chromosome = Chromosome(initial_shapes);
        }
        
        try {
            evaluate_fitness(initial_chromosome);
            population.push_back(initial_chromosome);
        }
        catch (...) {
            // If evaluation fails, continue with random population
        }
    }

    // Generate remaining random chromosomes
    // For circular sheets, use angle-based distribution for better visual spread
    for (size_t i = population.size(); i < params_.population_size; i++) {
        try {
        std::vector<TransformedShape> shapes;

            // For each shape in the initial layout, create a placement
            for (size_t shape_idx = 0; shape_idx < initial_shapes.size(); ++shape_idx) {
                try {
                    TransformedShape random_shape = (is_circular && i < static_cast<size_t>(params_.population_size * 0.6))
                        ? create_circular_distributed_shape(
                            initial_shapes[shape_idx], shape_idx, initial_shapes.size())
                        : create_random_shape(initial_shapes[shape_idx]);
            shapes.push_back(random_shape);
                }
                catch (...) {
                    // Skip this shape if creation fails
                    continue;
                }
            }

            if (shapes.empty()) {
                continue; // Skip this chromosome if no valid shapes
        }

        Chromosome chromosome(shapes);
            try {
        evaluate_fitness(chromosome);
            }
            catch (...) {
                // If fitness evaluation fails, set default values
                chromosome.utilization = 0.0;
                chromosome.fitness = -1e9;
                chromosome.overlap_penalty = 1e9;
                chromosome.boundary_penalty = 1e9;
            }
        population.push_back(chromosome);
        }
        catch (...) {
            // Skip this chromosome if initialization fails
            continue;
        }
    }

    // Sort by fitness (descending)
    std::sort(population.begin(), population.end());
    if (!population.empty())
        best_solution_ = population[0];

    return population;
}

TransformedShape GeneticAlgorithm::create_random_shape(const TransformedShape& base) {
    TransformedShape new_shape = base;

    try {
    // Random rotation
    uint32_t random_rotation = 0;
    if (base.allowed_rotations > 0) {
        random_rotation = rng_.rand_uint32() % base.allowed_rotations;
    }
    new_shape.set_rotation(random_rotation); // Pass rotation index, not angle
    }
    catch (...) {
        // Keep default rotation if setting fails
    }

    // Random position within the sheet
    double x = 0.0, y = 0.0;
    const Sheet& sheet = layout_.sheets[0];

    try {
    if (sheet.type == Sheet::ShapeType::Rectangle) {
        // For rectangular sheet
        double width = safe_to_double(sheet.width);
        double height = safe_to_double(sheet.height);

        // Get bounding box of the shape to ensure it fits
        auto bbox = new_shape.transformed.bbox();
        double shape_width = safe_to_double(bbox.xmax() - bbox.xmin());
        double shape_height = safe_to_double(bbox.ymax() - bbox.ymin());

        // Guard against shapes larger than sheet
        double max_x = std::max(0.0, width - shape_width);
        double max_y = std::max(0.0, height - shape_height);

        x = rng_.rand_double() * max_x;
        y = rng_.rand_double() * max_y;
    } else {
        // For circular sheet
        double radius = safe_to_double(sheet.diameter) / 2.0;
        double center_x = radius;
        double center_y = radius;

        // Get bounding box of the shape
        auto bbox = new_shape.transformed.bbox();
        double shape_width = safe_to_double(bbox.xmax() - bbox.xmin());
        double shape_height = safe_to_double(bbox.ymax() - bbox.ymin());

        // Generate random position within circle
        double max_r = std::max(0.0, radius - std::max(shape_width, shape_height) / 2.0);
        double r = rng_.rand_double() * max_r;
        double theta = rng_.rand_double() * 2.0 * geo::PI;

        x = center_x + r * std::cos(theta) - shape_width / 2.0;
        y = center_y + r * std::sin(theta) - shape_height / 2.0;

        // Ensure shape fits within circle bounding box
        x = std::max(0.0, std::min(x, 2.0 * radius - shape_width));
        y = std::max(0.0, std::min(y, 2.0 * radius - shape_height));
    }
    }
    catch (...) {
        // Fallback: place at origin if calculation fails
        x = 0.0;
        y = 0.0;
    }

    try {
    new_shape.set_translate(x, y);
    // Make sure transformed polygon is updated after move/rotation
    new_shape.update();
    }
    catch (...) {
        // If update fails, return shape as-is (may be invalid but won't crash)
    }
    return new_shape;
}

TransformedShape GeneticAlgorithm::create_circular_distributed_shape(
    const TransformedShape& base, size_t shape_index, size_t total_shapes) {
    TransformedShape new_shape = base;

    try {
        // Random rotation
        uint32_t random_rotation = 0;
        if (base.allowed_rotations > 0) {
            random_rotation = rng_.rand_uint32() % base.allowed_rotations;
        }
        new_shape.set_rotation(random_rotation);
    }
    catch (...) {
        // Keep default rotation if setting fails
    }

    const Sheet& sheet = layout_.sheets[0];
    if (sheet.type != Sheet::ShapeType::Circle) {
        // Fallback to random placement for non-circular sheets
        return create_random_shape(base);
    }

    try {
        double radius = safe_to_double(sheet.diameter) / 2.0;
        double center_x = radius;
        double center_y = radius;

        // Get bounding box of the shape
        auto bbox = new_shape.transformed.bbox();
        double shape_width = safe_to_double(bbox.xmax() - bbox.xmin());
        double shape_height = safe_to_double(bbox.ymax() - bbox.ymin());
        double shape_diagonal = std::sqrt(shape_width * shape_width + shape_height * shape_height);

        // Strategy: distribute shapes uniformly around the circle
        // Use uniform angle spacing for better visual distribution
        double angle_step = 2.0 * geo::PI / total_shapes;
        double base_angle = shape_index * angle_step;
        
        // Add small random variation (±10% of step) to avoid perfect symmetry
        double angle_variation = (rng_.rand_double() * 2.0 - 1.0) * angle_step * 0.1;
        base_angle += angle_variation;
        
        // Normalize angle to [0, 2π]
        base_angle = std::fmod(base_angle, 2.0 * geo::PI);
        if (base_angle < 0) base_angle += 2.0 * geo::PI;

        // Place shapes at different radii: distribute from outer to inner
        // Use a more uniform radial distribution
        double max_radius = radius - shape_diagonal * 0.5; // Leave margin
        double min_radius = radius * 0.4; // Don't go too close to center
        
        // Distribute radii more evenly: use shape_index to determine radius tier
        double r_tier = static_cast<double>(shape_index % 4) / 3.0; // 0, 1/3, 2/3, 1
        double target_r = min_radius + (max_radius - min_radius) * (0.4 + 0.6 * r_tier);
        
        // Add small random variation to radius (±5% of range)
        double r_variation = (rng_.rand_double() * 2.0 - 1.0) * (max_radius - min_radius) * 0.05;
        double r = std::max(min_radius, std::min(max_radius, target_r + r_variation));

        // Calculate position in Cartesian coordinates
        double x = center_x + r * std::cos(base_angle) - shape_width / 2.0;
        double y = center_y + r * std::sin(base_angle) - shape_height / 2.0;

        // Ensure shape fits within circle bounding box
        x = std::max(0.0, std::min(x, 2.0 * radius - shape_width));
        y = std::max(0.0, std::min(y, 2.0 * radius - shape_height));

        new_shape.set_translate(x, y);
        new_shape.update();
    }
    catch (...) {
        // Fallback to random placement if calculation fails
        return create_random_shape(base);
    }

    return new_shape;
}

// Compute area of intersection between two polygons (outer boundaries) using CGAL
static double intersection_area(const geo::Polygon_with_holes_2& a, const geo::Polygon_with_holes_2& b) {
    std::vector<geo::Polygon_with_holes_2> res;
    try {
        CGAL::intersection(a, b, std::back_inserter(res));
    }
    catch (...) {
        return 0.0;
    }
    double area = 0.0;
    for (auto& p : res) {
        area += polygon_area_double(p);
    }
    return std::max(0.0, area);
}

double GeneticAlgorithm::calculate_utilization(const Chromosome& chromosome) {
    // Use Polygon_set_2 to compute union area of all placed shapes clipped to sheet
    double union_area = 0.0;
    double sheet_area = 0.0;

    // Get sheet polygon and sheet area robustly
    geo::Polygon_with_holes_2 sheet_pwh = layout_.sheets[0].sheet;
    try {
        sheet_area = safe_to_double(geo::pwh_area(sheet_pwh));
    }
    catch (...) {
        // fallback: approximate by rectangle/diameter
        const Sheet& sheet = layout_.sheets[0];
        if (sheet.type == Sheet::ShapeType::Rectangle) {
            sheet_area = safe_to_double(sheet.width * sheet.height);
        }
        else {
            double radius = safe_to_double(sheet.diameter) / 2.0;
            sheet_area = geo::PI * radius * radius;
        }
    }
    if (sheet_area <= 0) return 0.0;

    // Build polygon set of clipped shapes
    geo::Polygon_set_2 ps(geo::traits);
    double sum_inside_area = 0.0;
    for (const auto& shape : chromosome.shapes) {
        // compute intersection between shape and sheet
        std::vector<geo::Polygon_with_holes_2> inters;
        try {
            CGAL::intersection(shape.transformed, sheet_pwh, std::back_inserter(inters));
        }
        catch (...) {
            // if intersection failed, skip this shape
            continue;
        }
        double inside_area = 0.0;
        for (auto& p : inters) {
            try { inside_area += safe_to_double(geo::pwh_area(p)); }
            catch (...) { /* ignore problematic polygon */ }
            // insert polygon pieces into polygon_set
            try { ps.join(p.outer_boundary()); }
            catch (...) { /* fallback: skip insertion */ }
        }
        sum_inside_area += inside_area;
    }

    // Extract polygons_with_holes from polygon_set and sum areas
    std::vector<geo::Polygon_with_holes_2> unions;
    try {
        ps.polygons_with_holes(std::back_inserter(unions));
    }
    catch (...) {
        unions.clear();
    }
    for (auto& p : unions) {
        try { union_area += safe_to_double(geo::pwh_area(p)); }
        catch (...) { /* ignore */ }
    }

    // union_area is the actual covered area inside sheet (no double count)
    double util = union_area / std::max(1e-12, sheet_area);
    if (util < 0) util = 0;
    if (util > 1) util = 1;
    return util;
}

void GeneticAlgorithm::evaluate_fitness(Chromosome& chromosome) {
    // Ensure transformed polygons reflect current translate/rotation
    for (auto& s : chromosome.shapes) {
        try {
        s.update();
        }
        catch (...) {
            // Skip update if it fails, shape may be invalid but won't crash
        }
    }

    // Get sheet polygon and sheet area robustly
    geo::Polygon_with_holes_2 sheet_pwh = layout_.sheets[0].sheet;
    double sheet_area = 0.0;
    try {
        sheet_area = safe_to_double(geo::pwh_area(sheet_pwh));
    }
    catch (...) {
        const Sheet& sheet = layout_.sheets[0];
        if (sheet.type == Sheet::ShapeType::Rectangle) {
            sheet_area = safe_to_double(sheet.width * sheet.height);
        }
        else {
            double radius = safe_to_double(sheet.diameter) / 2.0;
            sheet_area = geo::PI * radius * radius;
        }
    }
    if (sheet_area <= 0) sheet_area = 1.0;

    // Compute per-shape inside areas and accumulate into polygon_set for union
    geo::Polygon_set_2 ps(geo::traits);
    double sum_inside_area = 0.0;
    double oob_area = 0.0;

    for (const auto& s : chromosome.shapes) {
        double shape_area = 0.0;
        try { shape_area = safe_to_double(geo::pwh_area(s.transformed)); }
        catch (...) { shape_area = 0.0; }

        std::vector<geo::Polygon_with_holes_2> inters;
        try { CGAL::intersection(s.transformed, sheet_pwh, std::back_inserter(inters)); }
        catch (...) { inters.clear(); }

        double inside_area = 0.0;
        for (auto& p : inters) {
            try { inside_area += safe_to_double(geo::pwh_area(p)); }
            catch (...) { }
            // join outer boundary to polygon set; try-catch to avoid exceptions
            try { ps.join(p.outer_boundary()); }
            catch (...) { }
        }

        sum_inside_area += inside_area;
        double out_area = std::max(0.0, shape_area - inside_area);
        oob_area += out_area;
    }

    // Compute union area from polygon set
    std::vector<geo::Polygon_with_holes_2> unions;
    double union_area = 0.0;
    try { ps.polygons_with_holes(std::back_inserter(unions)); }
    catch (...) { unions.clear(); }
    for (auto& p : unions) {
        try { union_area += safe_to_double(geo::pwh_area(p)); }
        catch (...) { }
    }

    // overlap area is the extra counted area in sum_inside_area beyond union
    double overlap_area = std::max(0.0, sum_inside_area - union_area);

    chromosome.overlap_penalty = overlap_area;
    chromosome.boundary_penalty = oob_area;

    // utilization is union_area / sheet_area
    double utilization = union_area / std::max(1e-12, sheet_area);
    if (utilization < 0) utilization = 0;
    if (utilization > 1) utilization = 1;

    chromosome.utilization = utilization;

    // normalized penalty - heavily penalize overlaps
    // For overlaps, use a much stronger penalty to ensure GA avoids overlapping solutions
    double normalized_overlap = overlap_area / std::max(1e-9, sheet_area);
    double normalized_oob = oob_area / std::max(1e-9, sheet_area);
    // Overlap penalty is 10x more important than boundary penalty
    double total_penalty = 10.0 * normalized_overlap + normalized_oob;
    
    // For circular sheets: add angle distribution bonus to encourage uniform spread
    double angle_bonus = 0.0;
    if (layout_.sheets[0].type == Sheet::ShapeType::Circle && chromosome.shapes.size() > 2) {
        try {
            double radius = safe_to_double(layout_.sheets[0].diameter) / 2.0;
            double center_x = radius;
            double center_y = radius;
            
            // Collect angles of all shape centers
            std::vector<double> angles;
            angles.reserve(chromosome.shapes.size());
            
            for (const auto& s : chromosome.shapes) {
                auto bbox = s.transformed.bbox();
                double cx = safe_to_double((bbox.xmin() + bbox.xmax()) / 2.0);
                double cy = safe_to_double((bbox.ymin() + bbox.ymax()) / 2.0);
                double dx = cx - center_x;
                double dy = cy - center_y;
                double dist = std::sqrt(dx * dx + dy * dy);
                // Only consider shapes that are not too close to center (avoid center clustering)
                if (dist > radius * 0.2) {
                    double angle = std::atan2(dy, dx);
                    if (angle < 0) angle += 2.0 * geo::PI;
                    angles.push_back(angle);
                }
            }
            
            if (angles.size() > 1) {
                // Sort angles
                std::sort(angles.begin(), angles.end());
                
                // Calculate angle gaps and variance
                double sum_gaps = 0.0;
                double expected_gap = 2.0 * geo::PI / angles.size();
                
                for (size_t i = 0; i < angles.size(); ++i) {
                    double gap = angles[(i + 1) % angles.size()] - angles[i];
                    if (gap < 0) gap += 2.0 * geo::PI; // Wrap around
                    sum_gaps += std::abs(gap - expected_gap);
                }
                
                // Bonus: lower variance in angle gaps = better distribution
                // Increased bonus weight to 0.2 for better effect
                double angle_variance = sum_gaps / angles.size();
                angle_bonus = 0.2 * std::exp(-angle_variance * 3.0); // Stronger exponential decay
            }
        }
        catch (...) {
            // If angle calculation fails, no bonus
            angle_bonus = 0.0;
        }
    }
    
    chromosome.fitness = utilization + angle_bonus - params_.penalty_weight * total_penalty;

    // If there are any overlaps or out-of-bound areas, mark fitness as very bad
    if (normalized_overlap > 1e-6 || normalized_oob > 1e-6) {
        chromosome.fitness = -1e6;  // Very bad fitness for overlapping or outside solutions
    }

    if (std::isnan(chromosome.fitness) || std::isinf(chromosome.fitness)) chromosome.fitness = -1e9;
}

double GeneticAlgorithm::calculate_penalty(Chromosome& chromosome) {
    // Deprecated: we now calculate overlap/boundary penalties in evaluate_fitness
    return chromosome.overlap_penalty + chromosome.boundary_penalty;
}

bool GeneticAlgorithm::check_overlap(const TransformedShape& shape1, const TransformedShape& shape2) {
    // Quick bbox rejection
    auto b1 = shape1.transformed.outer_boundary().bbox();
    auto b2 = shape2.transformed.outer_boundary().bbox();
    if (!CGAL::do_overlap(b1, b2)) return false;

    // Check polygon intersection
    const auto& poly1 = shape1.transformed.outer_boundary();
    const auto& poly2 = shape2.transformed.outer_boundary();
    return CGAL::do_intersect(poly1, poly2);
}

bool GeneticAlgorithm::check_boundary(const TransformedShape& shape) {
    const Sheet& sheet = layout_.sheets[0];
    auto bbox = shape.transformed.bbox();
    
    double x_min = safe_to_double(bbox.xmin());
    double y_min = safe_to_double(bbox.ymin());
    double x_max = safe_to_double(bbox.xmax());
    double y_max = safe_to_double(bbox.ymax());

    if (sheet.type == Sheet::ShapeType::Rectangle) {
        double sheet_width = safe_to_double(sheet.width);
        double sheet_height = safe_to_double(sheet.height);
        
        return x_min >= 0.0 && y_min >= 0.0 && x_max <= sheet_width && y_max <= sheet_height;
    } else {
        double radius = safe_to_double(sheet.diameter) / 2.0;
        double center_x = radius;
        double center_y = radius;
        
        // Check each corner is inside circle
        double corners[4][2] = {{x_min, y_min}, {x_max, y_min}, {x_max, y_max}, {x_min, y_max}};
        for (int i = 0; i < 4; ++i) {
            double dx = corners[i][0] - center_x;
            double dy = corners[i][1] - center_y;
            if (dx * dx + dy * dy > radius * radius) return false;
        }
        return true;
    }
}

std::vector<Chromosome> GeneticAlgorithm::select_parents(const std::vector<Chromosome>& population) {
    std::vector<Chromosome> parents;
    parents.reserve(population.size());

    // Tournament selection
    for (size_t i = 0; i < population.size(); ++i) {
        // Select 3 random individuals
        size_t idx1 = rng_.rand_uint32() % population.size();
        size_t idx2 = rng_.rand_uint32() % population.size();
        size_t idx3 = rng_.rand_uint32() % population.size();

        // Choose the one with highest fitness
        const Chromosome& candidate1 = population[idx1];
        const Chromosome& candidate2 = population[idx2];
        const Chromosome& candidate3 = population[idx3];

        // Use manual comparison to avoid initializer_list overload issues
        const Chromosome* winner = &candidate1;
        if (candidate2.fitness > winner->fitness) winner = &candidate2;
        if (candidate3.fitness > winner->fitness) winner = &candidate3;
        parents.push_back(*winner);
    }

    return parents;
}

Chromosome GeneticAlgorithm::crossover(const Chromosome& parent1, const Chromosome& parent2) {
    if (rng_.rand_double() > params_.crossover_rate) {
        return rng_.rand_double() < 0.5 ? parent1 : parent2;
    }

    if (parent1.shapes.size() != parent2.shapes.size()) {
        return parent1;
    }

    // Single-point crossover
    size_t crossover_point = rng_.rand_uint32() % parent1.shapes.size();
    
    std::vector<TransformedShape> child_shapes;
    child_shapes.reserve(parent1.shapes.size());
    
    // Take first part from parent1
    for (size_t i = 0; i < crossover_point; ++i) {
        child_shapes.push_back(parent1.shapes[i]);
    }
    
    // Take second part from parent2
    for (size_t i = crossover_point; i < parent2.shapes.size(); ++i) {
        child_shapes.push_back(parent2.shapes[i]);
    }

    // ensure shapes updated
    for (auto& s : child_shapes) {
        try {
            s.update();
        }
        catch (...) {
            // If update fails, keep shape as-is
        }
    }

    return Chromosome(child_shapes);
}

void GeneticAlgorithm::mutate(Chromosome& chromosome) {
    const Sheet& sheet = layout_.sheets[0];
    bool is_circular = (sheet.type == Sheet::ShapeType::Circle);
    
    if (is_circular) {
        // For circular sheets: mutate in polar coordinates (angle + radius)
        double radius = safe_to_double(sheet.diameter) / 2.0;
        double center_x = radius;
        double center_y = radius;
        
        for (auto& shape : chromosome.shapes) {
            if (rng_.rand_double() <= params_.mutation_rate) {
                try {
                    // Get current position
                    double current_x = shape.get_translate_double_x();
                    double current_y = shape.get_translate_double_y();
                    
                    // Convert to polar coordinates relative to circle center
                    double dx = current_x + shape.transformed.bbox().x_span() / 2.0 - center_x;
                    double dy = current_y + shape.transformed.bbox().y_span() / 2.0 - center_y;
                    double current_r = std::sqrt(dx * dx + dy * dy);
                    double current_theta = std::atan2(dy, dx);
                    if (current_theta < 0) current_theta += 2.0 * geo::PI;
                    
                    // Mutate angle (small variation, ±5 degrees)
                    double angle_mutation = (rng_.rand_double() * 2.0 - 1.0) * (5.0 * geo::PI / 180.0);
                    double new_theta = current_theta + angle_mutation;
                    new_theta = std::fmod(new_theta, 2.0 * geo::PI);
                    if (new_theta < 0) new_theta += 2.0 * geo::PI;
                    
                    // Mutate radius (small variation, ±2% of radius)
                    double radius_mutation = (rng_.rand_double() * 2.0 - 1.0) * radius * 0.02;
                    double new_r = std::max(radius * 0.1, std::min(radius * 0.95, current_r + radius_mutation));
                    
                    // Convert back to Cartesian coordinates
                    auto bbox = shape.transformed.bbox();
                    double shape_width = safe_to_double(bbox.xmax() - bbox.xmin());
                    double shape_height = safe_to_double(bbox.ymax() - bbox.ymin());
                    
                    double new_x = center_x + new_r * std::cos(new_theta) - shape_width / 2.0;
                    double new_y = center_y + new_r * std::sin(new_theta) - shape_height / 2.0;
                    
                    // Ensure within bounds
                    new_x = std::max(0.0, std::min(new_x, 2.0 * radius - shape_width));
                    new_y = std::max(0.0, std::min(new_y, 2.0 * radius - shape_height));
                    
                    shape.set_translate(new_x, new_y);
                    shape.update();
                }
                catch (...) {
                    // If mutation fails, skip this shape
                }
            }
            
            if (rng_.rand_double() <= params_.mutation_rate) {
                // Mutate rotation
                if (shape.allowed_rotations > 0) {
                    try {
                        uint32_t new_rotation = rng_.rand_uint32() % shape.allowed_rotations;
                        shape.set_rotation(new_rotation);
                        shape.update();
                    }
                    catch (...) {
                        // Skip rotation mutation if it fails
                    }
                }
            }
        }
    } else {
        // For rectangular sheets: use Cartesian mutation (original logic)
    double sheet_w = safe_to_double(sheet.width);
        double sheet_h = safe_to_double(sheet.height);
    double diag = std::sqrt(sheet_w * sheet_w + sheet_h * sheet_h);
    double max_offset = std::max(1e-3, diag * 0.02); // 2% of diagonal

    for (auto& shape : chromosome.shapes) {
        if (rng_.rand_double() <= params_.mutation_rate) {
                try {
            // Mutate position scaled by sheet size
            double x_offset = (rng_.rand_double() * 2.0 - 1.0) * max_offset;
            double y_offset = (rng_.rand_double() * 2.0 - 1.0) * max_offset;
            
            // Get current position directly from translate properties
            double current_x = shape.get_translate_double_x();
            double current_y = shape.get_translate_double_y();
            
            // Update position
            shape.set_translate(current_x + x_offset, current_y + y_offset);
            shape.update();
                }
                catch (...) {
                    // Skip if mutation fails
                }
        }
        
        if (rng_.rand_double() <= params_.mutation_rate) {
            // Mutate rotation
            if (shape.allowed_rotations > 0) {
                    try {
                uint32_t new_rotation = rng_.rand_uint32() % shape.allowed_rotations;
                shape.set_rotation(new_rotation);
                shape.update();
                    }
                    catch (...) {
                        // Skip rotation mutation if it fails
                    }
                }
            }
        }
    }
}

bool GeneticAlgorithm::is_converged() {
    if (current_generation_ < params_.convergence_generations) {
        return false;
    }

    // Calculate average best fitness over the last N generations
    double sum = 0.0;
    size_t window_size = std::min(params_.convergence_generations, size_t(100));
    
    for (size_t i = 0; i < window_size; ++i) {
        sum += best_fitness_history_[(current_generation_ - i) % 100];
    }

    double avg_fitness = sum / window_size;

    // Check if the average fitness has not improved significantly
    return (best_solution_.fitness - avg_fitness) < params_.convergence_threshold;
}

Chromosome GeneticAlgorithm::optimize(volatile bool* requestQuit, std::function<void(const Chromosome&)> progress_callback) {
    // Initialize population
    std::vector<Chromosome> population = initialize_population();

    // Check if population is empty
    if (population.empty()) {
        return best_solution_;
    }

    for (current_generation_ = 0; current_generation_ < params_.max_generations; ++current_generation_) {
        // Check if optimization should be stopped
        if (requestQuit && *requestQuit) {
            break;
        }

        // Select parents
        std::vector<Chromosome> parents = select_parents(population);

        // Create new generation through crossover and mutation
        std::vector<Chromosome> new_generation;
        new_generation.reserve(params_.population_size);

        // Elitism: keep the best individuals
        size_t elitism_count = static_cast<size_t>(params_.elitism_rate * params_.population_size);
        for (size_t i = 0; i < elitism_count && i < population.size(); ++i) {
            new_generation.push_back(population[i]);
        }

        // Generate the rest of the new generation
        while (new_generation.size() < params_.population_size) {
            size_t parent1_idx = rng_.rand_uint32() % parents.size();
            size_t parent2_idx = rng_.rand_uint32() % parents.size();

            Chromosome child = crossover(parents[parent1_idx], parents[parent2_idx]);
            mutate(child);

            new_generation.push_back(child);
        }

        // Replace old population with new generation
        population = std::move(new_generation);

        // Evaluate fitness for all chromosomes
        for (auto& chromosome : population) {
            evaluate_fitness(chromosome);
        }

        // Sort by fitness
        std::sort(population.begin(), population.end());

        // Update best solution
        if (population[0].fitness > best_solution_.fitness || current_generation_ == 0) {
            best_solution_ = population[0];
        }

        // Store best fitness for convergence checking
        best_fitness_history_[current_generation_ % 100] = best_solution_.fitness;

        // Report progress periodically (less frequently to reduce overhead)
        if (progress_callback && (current_generation_ % 20 == 0)) {
            progress_callback(best_solution_);
        }

        // Check for convergence
        if (current_generation_ >= params_.convergence_generations && is_converged()) {
            break;
        }
    }

    return best_solution_;
}

// Integration function
void run_ga_optimization(Layout& layout, volatile bool* requestQuit,
                         std::function<void(const Layout&)> progress_callback) {
    // Strict validation: ensure layout is valid before starting GA
    if (layout.sheets.empty() || layout.sheet_parts.empty()) {
        std::clog << "GA: layout has no sheets or sheet_parts" << std::endl;
        return;
    }
    if (layout.sheet_parts[0].empty()) {
        std::clog << "GA: sheet_parts[0] is empty, cannot initialize population" << std::endl;
        return;
    }
    if (layout.poly_num == 0) {
        std::clog << "GA: poly_num is 0" << std::endl;
        return;
    }

    try {
    GeneticAlgorithm ga(layout);

        // Set GA parameters optimized for circular sheet nesting
        // Reduced population size for faster execution, increased penalty to avoid overlaps
    GeneticAlgorithm::Parameters params;
        params.population_size = 50;  // Reduced from 100 for speed
        params.max_generations = 300;  // Reduced from 500
    params.crossover_rate = 0.8;
        params.mutation_rate = 0.15;  // Slightly reduced to reduce invalid mutations
        params.elitism_rate = 0.15;   // Increased to preserve good solutions
    params.convergence_threshold = 0.001;
        params.convergence_generations = 30;
        params.penalty_weight = 10.0;  // Increased from 0.5 to heavily penalize overlaps
    ga.set_parameters(params);

    // Wrap GA progress to update layout and run local minimizer for feasibility
        // Accept solutions with small overlaps and try to fix them with minimize_overlap
        double last_best_util = -1.0;
        auto progress_wrapper = [&layout, &progress_callback, requestQuit, &last_best_util](const Chromosome& chromosome) {
            if (!layout.sheet_parts.empty() && !chromosome.shapes.empty()) {
                // Accept solutions if utilization improved (even with small overlaps)
                // We'll fix overlaps with minimize_overlap
                if (chromosome.utilization > last_best_util + 0.001) {
            layout.sheet_parts[0] = chromosome.shapes;
            layout.best_utilization = chromosome.utilization;
            layout.update_cur_length();
                    last_best_util = chromosome.utilization;

                    // Always try to refine to feasible layout (remove overlaps)
            try {
                bool feasible = minimize_overlap(layout, requestQuit);
                if (feasible) {
                    layout.best_result = layout.sheet_parts;
                    if (layout.best_length < FT(0)) {
                        layout.best_length = layout.cur_length;
                    } else {
                        layout.best_length = (std::min)(layout.best_length, layout.cur_length);
                    }
                            // For circular sheets, calculate utilization based on area only
                            if (layout.sheets[0].type == Sheet::ShapeType::Circle) {
                                double radius = safe_to_double(layout.sheets[0].diameter) / 2.0;
                                double sheet_area = geo::PI * radius * radius;
                                layout.best_utilization = safe_to_double(layout.area / sheet_area);
                            } else {
                    FT effective_height = layout.sheets[0].height;
                    layout.best_utilization = safe_to_double(layout.area / (layout.best_length * effective_height));
                            }
                }
            }
            catch (...) {
            }
            if (progress_callback) progress_callback(layout);
                }
        }
    };

    try {
        Chromosome best = ga.optimize(requestQuit, progress_wrapper);

            // Always apply the best solution found, and try to fix overlaps
            if (!layout.sheet_parts.empty() && !best.shapes.empty()) {
            layout.sheet_parts[0] = best.shapes;
            layout.update_cur_length();
                
                // Always try to remove overlaps with minimize_overlap
                bool feasible = false;
                try {
                    feasible = minimize_overlap(layout, requestQuit);
                }
                catch (...) {
                    feasible = false;
                }
                
                if (feasible) {
                    // Successfully removed overlaps
                    layout.best_result = layout.sheet_parts;
                    if (layout.best_length < FT(0)) {
                        layout.best_length = layout.cur_length;
                    } else {
                        layout.best_length = (std::min)(layout.best_length, layout.cur_length);
                    }
                    // For circular sheets, calculate utilization based on area only
                    if (layout.sheets[0].type == Sheet::ShapeType::Circle) {
                        double radius = safe_to_double(layout.sheets[0].diameter) / 2.0;
                        double sheet_area = geo::PI * radius * radius;
                        layout.best_utilization = safe_to_double(layout.area / sheet_area);
                    } else {
                    FT effective_height = layout.sheets[0].height;
                        layout.best_utilization = safe_to_double(layout.area / (layout.best_length * effective_height));
                    }
                }
                else {
                    // If minimize_overlap failed, still use the best solution we found
                    // Calculate utilization based on actual union area (may have overlaps)
                    layout.best_result = layout.sheet_parts;
                    if (layout.best_length < FT(0)) {
                        layout.best_length = layout.cur_length;
                    }
                    // Use the utilization from chromosome (which accounts for overlaps)
                layout.best_utilization = best.utilization;
            }
            layout.update_cur_length();
            if (progress_callback) progress_callback(layout);
        }
    }
    catch (const std::exception& e) {
        std::clog << "Error during GA optimization: " << e.what() << std::endl;
    }
    catch (...) {
        std::clog << "Unknown error during GA optimization" << std::endl;
        }
    }
    catch (const std::exception& e) {
        std::clog << "Error initializing GA: " << e.what() << std::endl;
    }
    catch (...) {
        std::clog << "Unknown error initializing GA" << std::endl;
    }
}

} // namespace nesting