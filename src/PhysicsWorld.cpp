#include "PhysicsWorld.h"

/**
 * PhysicsWorld implementation.
 *
 * BUG 2: Gravity is applied in the WRONG direction (+Y instead of -Y).
 * The body accelerates upward instead of falling downward.
 * After 60 steps at dt=1/60 a body at (0,10,0) reaches ~14.9 instead of ~5.095.
 * This causes rigid_body_integration to fail.
 */

void PhysicsWorld::addBody(RigidBody& body) {
    _bodies.push_back(&body);
}

void PhysicsWorld::step(float deltaTime) {
    for (auto* body : _bodies) {
        if (body->isStatic) continue;

        // BUG 2: += instead of -= causes upward acceleration
        body->vy += GRAVITY * deltaTime;

        body->x += body->vx * deltaTime;
        body->y += body->vy * deltaTime;
        body->z += body->vz * deltaTime;
    }
}

const std::vector<RigidBody*>& PhysicsWorld::getBodies() const {
    return _bodies;
}
