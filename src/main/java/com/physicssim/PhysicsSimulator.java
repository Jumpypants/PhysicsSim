package com.physicssim;

import java.util.ArrayList;
import java.util.List;

/**
 * PhysicsSimulator handles collision detection and response for rigid bodies.
 * Uses vertex-to-edge collision detection and impulse-based collision response.
 */
public class PhysicsSimulator {
    private static final double COLLISION_EPSILON = 1e-6;
    private static final int MAX_COLLISIONS_PER_FRAME = 100;
    private static final double CONTACT_SKIN = 0.05;
    // Below this approach speed, restitution is set to zero so slow contacts settle rather than bounce.
    private static final double RESTITUTION_VELOCITY_THRESHOLD = 0.5;

    /**
     * Simulates a single time step, handling collisions.
     * Subdivides the time step into smaller intervals whenever a collision occurs.
     *
     * @param rigidBodies List of rigid bodies to simulate
     * @param timeRemaining Time step to simulate (in seconds)
     */
    public void simulateTimeStep(List<RigidBody> rigidBodies, double timeRemaining) {
        for (RigidBody body : rigidBodies) body.clearTemporaryForces();
        detectAndApplyFriction(rigidBodies, timeRemaining);

        // Phase 1: advance time, resolving CCD collisions as they occur
        int collisionCount = 0;
        while (timeRemaining > COLLISION_EPSILON && collisionCount < MAX_COLLISIONS_PER_FRAME) {
            List<CollisionInfo> nextCollisions = findNextCollisions(rigidBodies, timeRemaining);

            if (nextCollisions.isEmpty()) {
                for (RigidBody body : rigidBodies) body.stepTime(timeRemaining);
                timeRemaining = 0;
                break;
            }

            double collisionTime = nextCollisions.get(0).collisionTime;
            for (RigidBody body : rigidBodies) body.stepTime(collisionTime);
            for (CollisionInfo collision : nextCollisions) {
                handleCollision(collision.bodyA, collision.bodyB,
                                collision.vertexA, collision.collisionNormal);
            }
            timeRemaining -= collisionTime;
            collisionCount++;
        }

        // If MAX_COLLISIONS_PER_FRAME was hit, consume whatever time remains so the
        // clock always advances by the full step.
        if (timeRemaining > COLLISION_EPSILON) {
            for (RigidBody body : rigidBodies) body.stepTime(timeRemaining);
        }

        // Phase 2: resolve ALL penetrations left over after time has advanced.
        // Loop until clean (or a safety cap), so multi-body pile-ups are fully resolved
        // in a single step rather than one body per step.
        int overlapIter = 0;
        while (checkAndResolveOverlap(rigidBodies) && overlapIter++ < 20);
    }

    /**
     * Finds the next collision that will occur within the time remaining.
     * Tests all vertex-to-edge pairs.
     *
     * @param rigidBodies List of rigid bodies
     * @param timeRemaining Maximum time to look ahead
     * @return CollisionInfo of the next collision, or null if no collision
     */
    private List<CollisionInfo> findNextCollisions(List<RigidBody> rigidBodies, double timeRemaining) {
        double earliestTime = Double.MAX_VALUE;
        List<CollisionInfo> simultaneous = new ArrayList<>();

        for (int i = 0; i < rigidBodies.size(); i++) {
            for (int j = i + 1; j < rigidBodies.size(); j++) {
                RigidBody bodyA = rigidBodies.get(i);
                RigidBody bodyB = rigidBodies.get(j);

                CollisionInfo colAB = findVertexEdgeCollision(bodyA, bodyB, timeRemaining);
                CollisionInfo colBA = findVertexEdgeCollision(bodyB, bodyA, timeRemaining);

                // Edge-edge contact: both directions fire simultaneously with conflicting normals.
                // Applying both impulses produces erratic results, so keep only the one whose
                // normal most strongly opposes the current approach velocity.
                if (colAB != null && colBA != null &&
                        Math.abs(colAB.collisionTime - colBA.collisionTime) <= COLLISION_EPSILON) {
                    Vector2D relVel = bodyA.getVelocity().subtract(bodyB.getVelocity());
                    double approachAB = relVel.dot(colAB.collisionNormal);
                    double approachBA = relVel.scale(-1).dot(colBA.collisionNormal);
                    if (approachAB <= approachBA) {
                        colBA = null;
                    } else {
                        colAB = null;
                    }
                }

                for (CollisionInfo collision : new CollisionInfo[]{colAB, colBA}) {
                    if (collision == null) continue;

                    if (collision.collisionTime < earliestTime - COLLISION_EPSILON) {
                        earliestTime = collision.collisionTime;
                        simultaneous.clear();
                        simultaneous.add(collision);
                    } else if (collision.collisionTime <= earliestTime + COLLISION_EPSILON) {
                        simultaneous.add(collision);
                    }
                }
            }
        }

        return simultaneous;
    }

    /**
     * Finds the next vertex-to-edge collision between two bodies.
     * Checks all vertices of bodyA against all edges of bodyB.
     *
     * @param bodyA Body whose vertices are tested
     * @param bodyB Body whose edges are tested against
     * @param timeRemaining Maximum time to look ahead
     * @return CollisionInfo if a collision is found, null otherwise
     */
    private CollisionInfo findVertexEdgeCollision(RigidBody bodyA, RigidBody bodyB, double timeRemaining) {
        Vector2D[] verticesA = bodyA.getVertices();
        Vector2D[] verticesB = bodyB.getVertices();

        double earliestTime = Double.MAX_VALUE;
        Vector2D earliestNormal = null;
        List<Vector2D> contactVertices = new ArrayList<>();

        for (int v = 0; v < verticesA.length; v++) {
            Vector2D vertexA = verticesA[v];

            for (int e = 0; e < verticesB.length; e++) {
                Vector2D edgeStart = verticesB[e];
                Vector2D edgeEnd = verticesB[(e + 1) % verticesB.length];

                CollisionInfo collision = findVertexEdgeCollisionTime(
                    bodyA, bodyB, vertexA, edgeStart, edgeEnd, timeRemaining
                );

                if (collision == null) continue;

                if (collision.collisionTime < earliestTime - COLLISION_EPSILON) {
                    // Strictly earlier: start a new contact manifold
                    earliestTime = collision.collisionTime;
                    earliestNormal = collision.collisionNormal;
                    contactVertices.clear();
                    contactVertices.add(vertexA);
                } else if (collision.collisionTime <= earliestTime + COLLISION_EPSILON) {
                    // Simultaneous: add to the contact manifold (avoid duplicates)
                    if (earliestNormal == null) {
                        earliestNormal = collision.collisionNormal;
                        earliestTime = collision.collisionTime;
                    }
                    boolean duplicate = false;
                    for (Vector2D cv : contactVertices) {
                        if (cv == vertexA) { duplicate = true; break; }
                    }
                    if (!duplicate) contactVertices.add(vertexA);
                }
            }
        }

        if (contactVertices.isEmpty()) return null;

        // Average simultaneous contact vertices into a single contact point to prevent
        // spurious torque on symmetric face-face collisions.
        double sumX = 0, sumY = 0;
        for (Vector2D v : contactVertices) { sumX += v.getX(); sumY += v.getY(); }
        Vector2D avgContact = new Vector2D(sumX / contactVertices.size(), sumY / contactVertices.size());

        return new CollisionInfo(bodyA, bodyB, avgContact, earliestNormal, earliestTime);
    }

    /**
     * Calculates the time at which a vertex (from bodyA) collides with an edge (from bodyB).
     * Uses relative motion to solve for collision time.
     *
     * @param bodyA Body containing the vertex
     * @param bodyB Body containing the edge
     * @param vertexBody Body-space position of vertex
     * @param edgeStart Body-space start of edge in bodyB
     * @param edgeEnd Body-space end of edge in bodyB
     * @param timeRemaining Maximum time to check
     * @return CollisionInfo if collision occurs, null otherwise
     */
    private CollisionInfo findVertexEdgeCollisionTime(RigidBody bodyA, RigidBody bodyB,
                                                       Vector2D vertexBody,
                                                       Vector2D edgeStart, Vector2D edgeEnd,
                                                       double timeRemaining) {
         // Convert body-space vertices to world-space
        Vector2D vertexWorldA = bodySpaceToWorldSpace(bodyA, vertexBody);
        Vector2D edgeStartWorldB = bodySpaceToWorldSpace(bodyB, edgeStart);
        Vector2D edgeEndWorldB = bodySpaceToWorldSpace(bodyB, edgeEnd);

        // Edge vector
        Vector2D edgeVector = edgeEndWorldB.subtract(edgeStartWorldB);
        double edgeMagnitude = edgeVector.magnitude();

        // Skip degenerate edges
        if (edgeMagnitude < COLLISION_EPSILON) {
            return null;
        }

        // Edge normal pointing to the right of the edge (outward for CCW polygons)
        // Rotate edge vector 90° clockwise: (ex, ey) -> (ey, -ex)
        Vector2D edgeNormal = new Vector2D(edgeVector.getY(), -edgeVector.getX()).scale(1.0 / edgeMagnitude);

        // Get velocities at the vertex and edge
        Vector2D vertexVelA = getPointVelocity(bodyA, vertexBody);
        Vector2D edgePointVelB = getPointVelocity(bodyB, edgeStart);

        // Relative velocity (how fast the vertex approaches the edge)
        Vector2D relativeVel = vertexVelA.subtract(edgePointVelB);
        double normalVel = relativeVel.dot(edgeNormal);

        // Check initial separation (positive = outside, negative = inside)
        Vector2D toVertex = vertexWorldA.subtract(edgeStartWorldB);
        double initialDistance = toVertex.dot(edgeNormal);

        // Case 1: Outside and moving away -> No collision
        if (initialDistance > COLLISION_EPSILON && normalVel >= -COLLISION_EPSILON) {
            return null;
        }

        // Case 2: Outside and moving parallel -> No collision
        if (Math.abs(normalVel) < COLLISION_EPSILON) {
            return null;
        }

        // Case 3: Moving towards the edge (normalVel < 0)
        double timeToCollision = -initialDistance / normalVel;

        // Check if collision happens within time remaining
        if (timeToCollision < COLLISION_EPSILON || timeToCollision > timeRemaining + COLLISION_EPSILON) {
            return null;
        }

        // Verify the collision point is on the edge segment
        Vector2D vertexAtCollision = vertexWorldA.add(vertexVelA.scale(timeToCollision));
        Vector2D edgePointVelEnd = getPointVelocity(bodyB, edgeEnd);
        Vector2D edgeStartAtCollision = edgeStartWorldB.add(edgePointVelB.scale(timeToCollision));
        Vector2D edgeEndAtCollision = edgeEndWorldB.add(edgePointVelEnd.scale(timeToCollision));

        // Project collision point onto edge at collision time
        Vector2D edgeVecAtCollision = edgeEndAtCollision.subtract(edgeStartAtCollision);
        Vector2D toCollision = vertexAtCollision.subtract(edgeStartAtCollision);
        double edgeLenSq = edgeVecAtCollision.dot(edgeVecAtCollision);

        if (edgeLenSq < COLLISION_EPSILON) {
            return null;
        }

        double t = toCollision.dot(edgeVecAtCollision) / edgeLenSq;

        if (t < -COLLISION_EPSILON || t > 1.0 + COLLISION_EPSILON) {
            return null;
        }

        return new CollisionInfo(bodyA, bodyB, vertexBody, edgeNormal, timeToCollision);
    }

    /**
     * Converts a point from body-space to world-space.
     */
    private Vector2D bodySpaceToWorldSpace(RigidBody body, Vector2D bodyPoint) {
        Vector2D rotated = bodyPoint.rotate(body.getWorldOrientation());
        return rotated.add(body.getWorldPos());
    }

    /**
     * Gets the velocity of a point on a rigid body (accounting for both linear and angular motion).
     */
    private Vector2D getPointVelocity(RigidBody body, Vector2D bodySpacePoint) {
        Vector2D worldPoint = bodySpaceToWorldSpace(body, bodySpacePoint);
        Vector2D toPoint = worldPoint.subtract(body.getWorldPos());

        // v = v_linear + ω × r (in 2D: ω_z × r = (-ω_z * ry, ω_z * rx))
        double angularContribution_x = -body.getAngularVelocity() * toPoint.getY();
        double angularContribution_y = body.getAngularVelocity() * toPoint.getX();

        return body.getVelocity().add(new Vector2D(angularContribution_x, angularContribution_y));
    }

    /**
     * Handles a collision using impulse-based collision response.
     * Conserves momentum and uses coefficient of restitution.
     */
    private void handleCollision(RigidBody bodyA, RigidBody bodyB, Vector2D vertexA, Vector2D normal) {
        // Get current velocities at collision point
        Vector2D velocityA = getPointVelocity(bodyA, vertexA);

        // For bodyB, find the closest point on any edge to the vertex position
        Vector2D[] verticesB = bodyB.getVertices();
        Vector2D collisionPointB = vertexA; // Default: use vertex position (will be updated)
        double minDist = Double.MAX_VALUE;

        // Find the closest point on bodyB to the collision
        Vector2D vertexAWorld = bodySpaceToWorldSpace(bodyA, vertexA);
        for (int i = 0; i < verticesB.length; i++) {
            Vector2D edgeStart = verticesB[i];
            Vector2D edgeEnd = verticesB[(i + 1) % verticesB.length];

            Vector2D edgeStartWorld = bodySpaceToWorldSpace(bodyB, edgeStart);
            Vector2D edgeEndWorld = bodySpaceToWorldSpace(bodyB, edgeEnd);

            Vector2D edgeVec = edgeEndWorld.subtract(edgeStartWorld);
            double edgeLenSq = edgeVec.dot(edgeVec);

            if (edgeLenSq > COLLISION_EPSILON) {
                double t = vertexAWorld.subtract(edgeStartWorld).dot(edgeVec) / edgeLenSq;
                t = Math.max(0, Math.min(1, t));

                Vector2D closestPoint = edgeStartWorld.add(edgeVec.scale(t));
                double dist = vertexAWorld.subtract(closestPoint).magnitude();

                if (dist < minDist) {
                    minDist = dist;
                    // Convert closest point back to body-space
                    Vector2D relPos = closestPoint.subtract(bodyB.getWorldPos());
                    double rotAngle = -bodyB.getWorldOrientation();
                    collisionPointB = relPos.rotate(rotAngle);
                }
            }
        }

        Vector2D velocityB = getPointVelocity(bodyB, collisionPointB);

         // Relative velocity of approach
        Vector2D relativeVel = velocityA.subtract(velocityB);
        double relativeVelNormal = relativeVel.dot(normal);

        // If separating, skip (with small epsilon for numerical error tolerance)
        if (relativeVelNormal > 1e-6) {
            return;
        }

        // Use zero restitution for slow impacts so resting contacts settle instead of micro-bouncing.
        double e = (Math.abs(relativeVelNormal) < RESTITUTION_VELOCITY_THRESHOLD)
                   ? 0.0
                   : (bodyA.getCoefficientOfRestitution() + bodyB.getCoefficientOfRestitution()) / 2.0;

        // Get position vectors from body centers to collision point (in world-space)
        Vector2D r_A = bodySpaceToWorldSpace(bodyA, vertexA).subtract(bodyA.getWorldPos());
        Vector2D r_B = bodySpaceToWorldSpace(bodyB, collisionPointB).subtract(bodyB.getWorldPos());

        // Calculate impulse magnitude using the impulse-momentum equations
        // j = -(1 + e) * (v_rel · n) / (1/m_a + 1/m_b + (r_a × n)^2 / I_a + (r_b × n)^2 / I_b)

        // Static bodies contribute 0 to the inverse-mass terms (infinite effective mass)
        double invMa = bodyA.isStatic() ? 0.0 : 1.0 / bodyA.getMass();
        double invMb = bodyB.isStatic() ? 0.0 : 1.0 / bodyB.getMass();
        double invIa = bodyA.isStatic() ? 0.0 : 1.0 / bodyA.getMomentOfInertia();
        double invIb = bodyB.isStatic() ? 0.0 : 1.0 / bodyB.getMomentOfInertia();

        // Cross products (in 2D, a × b gives a scalar)
        double r_a_cross_n = r_A.getX() * normal.getY() - r_A.getY() * normal.getX();
        double r_b_cross_n = r_B.getX() * normal.getY() - r_B.getY() * normal.getX();

        double denominator = invMa + invMb +
                             (r_a_cross_n * r_a_cross_n) * invIa +
                             (r_b_cross_n * r_b_cross_n) * invIb;

        double j = -(1.0 + e) * relativeVelNormal / denominator;

        // Apply impulse: J = j * n
        Vector2D impulse = normal.scale(j);

        // Update velocities (static bodies ignore the impulse)
        if (!bodyA.isStatic()) {
            bodyA.setVelocity(bodyA.getVelocity().add(impulse.scale(invMa)));
            bodyA.setAngularVelocity(bodyA.getAngularVelocity() + r_a_cross_n * j * invIa);
        }
        if (!bodyB.isStatic()) {
            bodyB.setVelocity(bodyB.getVelocity().add(impulse.scale(-invMb)));
            bodyB.setAngularVelocity(bodyB.getAngularVelocity() - r_b_cross_n * j * invIb);
        }

    }

    private void detectAndApplyFriction(List<RigidBody> bodies, double dt) {
        for (int i = 0; i < bodies.size(); i++) {
            for (int j = i + 1; j < bodies.size(); j++) {
                applyFrictionForPair(bodies.get(i), bodies.get(j), dt);
                applyFrictionForPair(bodies.get(j), bodies.get(i), dt);
            }
        }
    }

    private void applyFrictionForPair(RigidBody bodyA, RigidBody bodyB, double dt) {
        if (bodyA.isStatic()) return;

        Vector2D[] vertsA = bodyA.getVertices();
        Vector2D[] vertsB = bodyB.getVertices();

        // Collect contact vertices and determine the dominant contact normal
        List<Vector2D> contactBodyVerts = new ArrayList<>();
        Vector2D contactNormal = null;
        double bestAbsDist = Double.MAX_VALUE;

        for (Vector2D vBody : vertsA) {
            Vector2D vWorld = bodySpaceToWorldSpace(bodyA, vBody);

            for (int e = 0; e < vertsB.length; e++) {
                Vector2D e0w = bodySpaceToWorldSpace(bodyB, vertsB[e]);
                Vector2D e1w = bodySpaceToWorldSpace(bodyB, vertsB[(e + 1) % vertsB.length]);

                Vector2D edgeVec = e1w.subtract(e0w);
                double edgeMag = edgeVec.magnitude();
                if (edgeMag < COLLISION_EPSILON) continue;

                // Outward edge normal (CW rotation for CCW polygon)
                Vector2D edgeNormal = new Vector2D(edgeVec.getY(), -edgeVec.getX()).scale(1.0 / edgeMag);

                // Signed distance: positive = outside, negative = inside
                double dist = vWorld.subtract(e0w).dot(edgeNormal);
                if (dist < -CONTACT_SKIN || dist > CONTACT_SKIN) continue;

                // Check vertex projects onto the edge segment (not past the endpoints)
                double along = vWorld.subtract(e0w).dot(edgeVec) / (edgeMag * edgeMag);
                if (along < 0 || along > 1) continue;

                contactBodyVerts.add(vBody);
                if (Math.abs(dist) < bestAbsDist) {
                    bestAbsDist = Math.abs(dist);
                    contactNormal = edgeNormal;
                }
                break; // One edge per vertex is enough
            }
        }

        if (contactBodyVerts.isEmpty() || contactNormal == null) return;

        // Average contact point in body A's body space
        double sx = 0, sy = 0;
        for (Vector2D v : contactBodyVerts) { sx += v.getX(); sy += v.getY(); }
        Vector2D avgContactA = new Vector2D(sx / contactBodyVerts.size(), sy / contactBodyVerts.size());

        // Normal force = component of permanent applied forces on A pressing into B.
        // Uses only permanent forces so friction doesn't depend on itself.
        Vector2D totalPermForce = new Vector2D(0, 0);
        for (Force f : bodyA.getAppliedForces()) totalPermForce = totalPermForce.add(f.getForceVector());
        double normalForce = -totalPermForce.dot(contactNormal); // Positive when pressing into B
        if (normalForce <= 0) return;

        // Relative tangential velocity at the contact point
        Vector2D velA = getPointVelocity(bodyA, avgContactA);
        Vector2D contactWorld = bodySpaceToWorldSpace(bodyA, avgContactA);
        Vector2D relToCenterB = contactWorld.subtract(bodyB.getWorldPos());
        Vector2D contactBBody = relToCenterB.rotate(-bodyB.getWorldOrientation());
        Vector2D velB = getPointVelocity(bodyB, contactBBody);

        Vector2D relVel = velA.subtract(velB);
        Vector2D tangentialRelVel = relVel.subtract(contactNormal.scale(relVel.dot(contactNormal)));
        double tangentSpeed = tangentialRelVel.magnitude();
        if (tangentSpeed < 1e-4) return; // No meaningful sliding

        Vector2D tangent = tangentialRelVel.scale(1.0 / tangentSpeed);
        double mu = (bodyA.getCoefficientOfKineticFriction() + bodyB.getCoefficientOfKineticFriction()) / 2.0;

        // Effective mass at the contact point in the tangential direction.
        // Caps the friction force so it can decelerate the sliding to zero in one step
        // but never overshoot — prevents oscillation of nearly-stationary bodies.
        Vector2D r_A = contactWorld.subtract(bodyA.getWorldPos());
        Vector2D r_B = contactWorld.subtract(bodyB.getWorldPos());
        double invMa = bodyA.isStatic() ? 0.0 : 1.0 / bodyA.getMass();
        double invMb = bodyB.isStatic() ? 0.0 : 1.0 / bodyB.getMass();
        double invIa = bodyA.isStatic() ? 0.0 : 1.0 / bodyA.getMomentOfInertia();
        double invIb = bodyB.isStatic() ? 0.0 : 1.0 / bodyB.getMomentOfInertia();
        double r_a_cross_t = r_A.getX() * tangent.getY() - r_A.getY() * tangent.getX();
        double r_b_cross_t = r_B.getX() * tangent.getY() - r_B.getY() * tangent.getX();
        double effMassDenom = invMa + invMb + r_a_cross_t * r_a_cross_t * invIa + r_b_cross_t * r_b_cross_t * invIb;
        double mEff = (effMassDenom > COLLISION_EPSILON) ? 1.0 / effMassDenom : 0.0;

        double frictionMagnitude = Math.min(mu * normalForce, tangentSpeed * mEff / dt);

        Vector2D frictionForce = tangent.scale(-frictionMagnitude);
        bodyA.applyTemporaryForce(new Force(frictionForce, avgContactA));

        // Reaction force on bodyB at the corresponding contact point
        if (!bodyB.isStatic()) {
            bodyB.applyTemporaryForce(new Force(frictionForce.scale(-1.0), contactBBody));
        }
    }

    // Detect and resolve all pairwise overlaps using full SAT.
    // Unlike the old vertex-in-polygon approach, SAT catches edge-edge intersections
    // where no vertex of A is inside B — the case that caused bodies to sink through each other.
    private boolean checkAndResolveOverlap(List<RigidBody> bodies) {
        boolean anyResolved = false;
        for (int i = 0; i < bodies.size(); i++) {
            RigidBody A = bodies.get(i);
            if (A.isStatic()) continue;
            Vector2D[] vertsA = A.getVertices();
            for (int j = 0; j < bodies.size(); j++) {
                if (i == j) continue;
                RigidBody B = bodies.get(j);
                Vector2D[] vertsB = B.getVertices();

                Vector2D[] worldA = new Vector2D[vertsA.length];
                Vector2D[] worldB = new Vector2D[vertsB.length];
                for (int k = 0; k < vertsA.length; k++) worldA[k] = bodySpaceToWorldSpace(A, vertsA[k]);
                for (int k = 0; k < vertsB.length; k++) worldB[k] = bodySpaceToWorldSpace(B, vertsB[k]);

                double[] sat = satPenetration(worldA, worldB, A.getWorldPos(), B.getWorldPos());
                if (sat == null) continue; // Separating axis found — no overlap

                double depth = sat[0];
                Vector2D resolveNormal = new Vector2D(sat[1], sat[2]);

                // Contact point: the most-penetrating vertices of A (smallest projection
                // onto resolveNormal, which points from B toward A).
                double minProj = Double.MAX_VALUE;
                for (Vector2D v : worldA) {
                    double p = v.dot(resolveNormal);
                    if (p < minProj) minProj = p;
                }
                double threshold = minProj + COLLISION_EPSILON;
                List<Vector2D> contactBodyVerts = new ArrayList<>();
                for (int k = 0; k < worldA.length; k++) {
                    if (worldA[k].dot(resolveNormal) <= threshold) contactBodyVerts.add(vertsA[k]);
                }
                if (contactBodyVerts.isEmpty()) contactBodyVerts.add(new Vector2D(0, 0));

                double sumX = 0, sumY = 0;
                for (Vector2D v : contactBodyVerts) { sumX += v.getX(); sumY += v.getY(); }
                Vector2D avgContact = new Vector2D(sumX / contactBodyVerts.size(), sumY / contactBodyVerts.size());

                // Split position correction proportionally to inverse mass so neither body
                // gets driven into a third object (e.g. hex pushing player through a wall).
                // Static bodies contribute zero inverse-mass and absorb no correction.
                double invMassA = 1.0 / A.getMass();
                double invMassB = B.isStatic() ? 0.0 : 1.0 / B.getMass();
                double totalInvMass = invMassA + invMassB;
                A.setWorldPos(A.getWorldPos().add(resolveNormal.scale(depth * invMassA / totalInvMass)));
                if (!B.isStatic()) B.setWorldPos(B.getWorldPos().subtract(resolveNormal.scale(depth * invMassB / totalInvMass)));

                Vector2D vel = A.getVelocity();
                double centerApproach  = vel.dot(resolveNormal);
                Vector2D contactVel    = getPointVelocity(A, avgContact);
                double contactApproach = contactVel.dot(resolveNormal);

                if (!A.isStatic() && centerApproach < 0 && contactApproach >= 0) {
                    A.setVelocity(vel.subtract(resolveNormal.scale(centerApproach)));
                    contactVel = getPointVelocity(A, avgContact);
                }

                if (contactVel.dot(resolveNormal) < 0) {
                    handleCollision(A, B, avgContact, resolveNormal);
                }
                anyResolved = true;
            }
        }
        return anyResolved;
    }

    // Full SAT overlap test for two convex polygons.
    // Tests every edge normal of both polygons. Returns {depth, nx, ny} — the minimum
    // translation vector that pushes worldA out of worldB — or null if a separating axis exists.
    // The returned normal points from B toward A.
    private double[] satPenetration(Vector2D[] worldA, Vector2D[] worldB,
                                    Vector2D centerA, Vector2D centerB) {
        double minDepth = Double.MAX_VALUE;
        double bestNX = 0, bestNY = 0;

        for (int pass = 0; pass < 2; pass++) {
            Vector2D[] poly = (pass == 0) ? worldA : worldB;
            for (int i = 0; i < poly.length; i++) {
                Vector2D e0 = poly[i];
                Vector2D e1 = poly[(i + 1) % poly.length];
                double ex = e1.getX() - e0.getX(), ey = e1.getY() - e0.getY();
                double mag = Math.sqrt(ex * ex + ey * ey);
                if (mag < COLLISION_EPSILON) continue;
                double nx = ey / mag, ny = -ex / mag;

                double minA = Double.MAX_VALUE, maxA = -Double.MAX_VALUE;
                for (Vector2D v : worldA) {
                    double p = v.getX() * nx + v.getY() * ny;
                    if (p < minA) minA = p;
                    if (p > maxA) maxA = p;
                }
                double minB = Double.MAX_VALUE, maxB = -Double.MAX_VALUE;
                for (Vector2D v : worldB) {
                    double p = v.getX() * nx + v.getY() * ny;
                    if (p < minB) minB = p;
                    if (p > maxB) maxB = p;
                }

                double overlap = Math.min(maxA, maxB) - Math.max(minA, minB);
                if (overlap <= 0) return null; // Separating axis — bodies do not overlap

                if (overlap < minDepth) {
                    minDepth = overlap;
                    // Flip so the normal points from B toward A
                    double dot = (centerA.getX() - centerB.getX()) * nx + (centerA.getY() - centerB.getY()) * ny;
                    double sign = (dot >= 0) ? 1.0 : -1.0;
                    bestNX = nx * sign;
                    bestNY = ny * sign;
                }
            }
        }

        return new double[]{minDepth, bestNX, bestNY};
    }

    /**
     * Internal class to store collision information.
     */
    private static class CollisionInfo {
        RigidBody bodyA;
        RigidBody bodyB;
        Vector2D vertexA; // Body-space position of vertex
        Vector2D collisionNormal;
        double collisionTime;

        CollisionInfo(RigidBody bodyA, RigidBody bodyB, Vector2D vertexA, Vector2D collisionNormal, double collisionTime) {
            this.bodyA = bodyA;
            this.bodyB = bodyB;
            this.vertexA = vertexA;
            this.collisionNormal = collisionNormal;
            this.collisionTime = collisionTime;
        }
    }
}
