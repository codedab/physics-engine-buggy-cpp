#include "ConstraintSolver.h"
#include <cmath>

/**
 * ConstraintSolver implementation.
 *
 * BUG 4: Baumgarte factor is 0.0f — no correction is ever applied.
 * Bodies remain at their initial positions regardless of iterations.
 * Two bodies starting 4.0 units apart with a 2.0-unit constraint will
 * still be 4.0 units apart after 10 iterations.
 * This causes constraint_solver_basic to fail.
 */

// BUG 4: Factor is 0.0 — constraint correction is permanently disabled
const float BAUMGARTE = 0.0f;  // should be 0.4f

void ConstraintSolver::addDistanceConstraint(RigidBody* a, RigidBody* b, float distance) {
    _constraints.push_back({a, b, distance});
}

void ConstraintSolver::solve(int iterations) {
    for (int iter = 0; iter < iterations; ++iter) {
        for (auto& c : _constraints) {
            if (!c.a || !c.b) continue;

            float dx = c.b->x - c.a->x;
            float dy = c.b->y - c.a->y;
            float dz = c.b->z - c.a->z;

            float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (dist < 1e-6f) continue;

            float error      = dist - c.distance;
            float correction = BAUMGARTE * error;  // BUG: always 0

            float nx = dx / dist;
            float ny = dy / dist;
            float nz = dz / dist;

            float invMassA = c.a->isStatic ? 0.0f : (1.0f / c.a->mass);
            float invMassB = c.b->isStatic ? 0.0f : (1.0f / c.b->mass);
            float totalInv = invMassA + invMassB;
            if (totalInv < 1e-9f) continue;

            float shareA = invMassA / totalInv;
            float shareB = invMassB / totalInv;

            // BUG: correction is 0, so nothing moves
            c.a->x += nx * correction * shareA;
            c.a->y += ny * correction * shareA;
            c.a->z += nz * correction * shareA;

            c.b->x -= nx * correction * shareB;
            c.b->y -= ny * correction * shareB;
            c.b->z -= nz * correction * shareB;
        }
    }
}
