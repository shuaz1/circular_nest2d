#pragma once
#include "layout.h"
#include "rand.h"
#include <random>
#include <vector>
#include <algorithm>
#include <memory>
#include <functional>

namespace nesting {

// Lightweight random number wrapper used by GA
namespace random {
    struct RandomGenerator {
        std::mt19937 engine;
        RandomGenerator() : engine(std::random_device{}()) {}

        uint32_t rand_uint32() {
            return engine();
        }

        double rand_double() {
            return std::generate_canonical<double, 53>(engine);
        }
    };
}

struct Chromosome {
    std::vector<TransformedShape> shapes; // Encoded solution
    double utilization; // Raw utilization (0..1)
    double fitness; // Penalized fitness = utilization - penalty
    double overlap_penalty; // Penalty for overlaps
    double boundary_penalty; // Penalty for boundary violations

    Chromosome() : utilization(0.0), fitness(0.0), overlap_penalty(0.0), boundary_penalty(0.0) {}
    Chromosome(const std::vector<TransformedShape>& _shapes)
        : shapes(_shapes), utilization(0.0), fitness(0.0), overlap_penalty(0.0), boundary_penalty(0.0) {}

    bool operator<(const Chromosome& other) const {
        return fitness > other.fitness; // Higher fitness comes first
    }
};

class GeneticAlgorithm {
public:
    GeneticAlgorithm(Layout& layout);

    // GA parameters
    struct Parameters {
        size_t population_size = 100;
        size_t max_generations = 500;
        double crossover_rate = 0.8;
        double mutation_rate = 0.2;
        double elitism_rate = 0.1;
        double convergence_threshold = 0.001;
        size_t convergence_generations = 50;
        // Penalty weight scales the raw penalty to keep fitness in reasonable range
        // Increased default from 0.1 to 0.5 to balance utilization vs normalized penalty
        double penalty_weight = 0.5;
    };

    void set_parameters(const Parameters& params);

    // Run the GA optimization
    Chromosome optimize(volatile bool* requestQuit, std::function<void(const Chromosome&)> progress_callback = nullptr);

    // Get the best solution found
    const Chromosome& get_best_solution() const;

private:
    Layout& layout_;
    Parameters params_;
    Chromosome best_solution_;
    double best_fitness_history_[100]; // For convergence checking
    size_t current_generation_;

    // Core GA functions
    std::vector<Chromosome> initialize_population();
    void evaluate_fitness(Chromosome& chromosome);
    double calculate_utilization(const Chromosome& chromosome);
    bool check_overlap(const TransformedShape& shape1, const TransformedShape& shape2);
    bool check_boundary(const TransformedShape& shape);

    // GA operators
    std::vector<Chromosome> select_parents(const std::vector<Chromosome>& population);
    Chromosome crossover(const Chromosome& parent1, const Chromosome& parent2);
    void mutate(Chromosome& chromosome);

    // Helper functions
    TransformedShape create_random_shape(const TransformedShape& base);
    TransformedShape create_circular_distributed_shape(const TransformedShape& base, 
                                                       size_t shape_index, 
                                                       size_t total_shapes);
    double calculate_penalty(Chromosome& chromosome);
    bool is_converged();

    // Random number generator
    random::RandomGenerator rng_;
};

// Integration function to be called from nesting module
void run_ga_optimization(Layout& layout, volatile bool* requestQuit,
                         std::function<void(const Layout&)> progress_callback = nullptr);

} // namespace nesting
