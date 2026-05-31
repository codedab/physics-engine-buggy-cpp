#include "AABB.h"

/**
 * AABB overlap test.
 *
 * BUG 1: Uses strict < and > instead of <= and >=.
 * Touching boxes that share exactly one boundary plane are NOT reported
 * as colliding. The harness places boxes with a 0.5-unit overlap on X —
 * this bug causes bvh_collision_detection to fail.
 */
bool overlaps(const AABB& a, const AABB& b) {
    return a.minX < b.maxX && a.maxX > b.minX &&
           a.minY < b.maxY && a.maxY > b.minY &&
           a.minZ < b.maxZ && a.maxZ > b.minZ;
}
