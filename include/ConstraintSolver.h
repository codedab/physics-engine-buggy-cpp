#pragma once

#include "RigidBody.h"
#include <vector>

/**
 * ConstraintSolver.h — Positional Constraint Solver
 *
 * Resolves distance constraints between pairs of rigid bodies using
 * Baumgarte stabilisation. The solver iterates a fixed number of times
 * per physics step.
 */
class ConstraintSolver {
public:
    ConstraintSolver() = default;
    ~ConstraintSolver() = default;

    /**
     * Add a distance constraint requiring bodies a and b to remain
     * exactly 'distance' units apart.
     */
    void addDistanceConstraint(RigidBody* a, RigidBody* b, float distance);

    /**
     * Solve all constraints for the given number of iterations using
     * Baumgarte stabilisation (factor 0.2).
     */
    void solve(int iterations);

private:
    struct DistanceConstraint {
        RigidBody* a;
        RigidBody* b;
        float distance;
    };

    std::vector<DistanceConstraint> _constraints;

    // BUG 4: Baumgarte factor is 0.0 — constraint correction is permanently disabled
    // Should be 0.4f for proper convergence
    static constexpr float BAUMGARTE = 0.0f;
};
