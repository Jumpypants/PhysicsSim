package com.physicssim;

import java.util.ArrayList;
import java.util.List;

/**
 * PhysicsSimulator handles collision detection and response for rigid bodies.
 * Uses vertex-to-edge collision detection and impulse-based collision response.
 *
 * Broad phase: a uniform spatial grid generates O(n) candidate pairs per step.
 * The existing bounding-circle check inside each loop acts as the precise filter.
 */
public class PhysicsSimulator {
    private static final double COLLISION_EPSILON = 1e-6;
    private static final int MAX_COLLISIONS_PER_FRAME = 100;
    private static final double CONTACT_SKIN = 0.05;
    private static final double RESTITUTION_VELOCITY_THRESHOLD = 0.5;
    private static final double OVERLAP_SLOP = 0.002;

    private final SpatialGrid grid = new SpatialGrid();

    /**
     * Simulates a single time step, handling collisions.
     * Subdivides the time step into smaller intervals whenever a collision occurs.
     */
    public void simulateTimeStep(List<RigidBody> rigidBodies, double timeRemaining) {
        double dt = timeRemaining;

        // Build spatial grid once per step; reuse candidates for all three broad-phase loops.
        grid.build(rigidBodies, maxVelocityExpansion(rigidBodies, timeRemaining));
        List<int[]> candidates = grid.candidatePairs(rigidBodies.size());

        for (RigidBody body : rigidBodies) body.clearTemporaryForces();
        detectAndApplyFriction(rigidBodies, timeRemaining, candidates);

        // Phase 1: advance time, resolving CCD collisions as they occur.
        int collisionCount = 0;
        while (timeRemaining > COLLISION_EPSILON && collisionCount < MAX_COLLISIONS_PER_FRAME) {
            List<CollisionInfo> nextCollisions = findNextCollisions(rigidBodies, timeRemaining, candidates);

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

        if (timeRemaining > COLLISION_EPSILON) {
            for (RigidBody body : rigidBodies) body.stepTime(timeRemaining);
        }

        // Phase 2: resolve all remaining penetrations.
        int overlapIter = 0;
        while (checkAndResolveOverlap(rigidBodies, candidates) && overlapIter++ < 20);

        for (RigidBody body : rigidBodies) body.updateSleepTimer(dt);
    }

    // ── Broad phase ───────────────────────────────────────────────────────────

    /** Maximum speed-based sweep expansion across all bodies for the CCD grid build. */
    private static double maxVelocityExpansion(List<RigidBody> bodies, double dt) {
        double maxSpeed = 0;
        for (RigidBody b : bodies) {
            Vector2D v = b.getVelocity();
            double s = v.getX() * v.getX() + v.getY() * v.getY();
            if (s > maxSpeed) maxSpeed = s;
        }
        return Math.sqrt(maxSpeed) * dt;
    }

    private static boolean boundingCirclesOverlap(RigidBody a, RigidBody b, double expansion) {
        double dx = a.getWorldPos().getX() - b.getWorldPos().getX();
        double dy = a.getWorldPos().getY() - b.getWorldPos().getY();
        double combined = a.getBoundingRadius() + b.getBoundingRadius() + expansion;
        return dx * dx + dy * dy <= combined * combined;
    }

    // ── CCD collision detection ───────────────────────────────────────────────

    private List<CollisionInfo> findNextCollisions(List<RigidBody> rigidBodies,
                                                    double timeRemaining,
                                                    List<int[]> candidates) {
        double earliestTime = Double.MAX_VALUE;
        List<CollisionInfo> simultaneous = new ArrayList<>();

        for (int[] pair : candidates) {
            RigidBody bodyA = rigidBodies.get(pair[0]);
            RigidBody bodyB = rigidBodies.get(pair[1]);

            if (bodyA.isSleeping() && bodyB.isSleeping()) continue;

            // Precise bounding-circle check with velocity expansion.
            Vector2D broadRelVel = bodyA.getVelocity().subtract(bodyB.getVelocity());
            double relSpeed = Math.sqrt(broadRelVel.getX() * broadRelVel.getX()
                                      + broadRelVel.getY() * broadRelVel.getY());
            if (!boundingCirclesOverlap(bodyA, bodyB, relSpeed * timeRemaining)) continue;

            CollisionInfo colAB = findVertexEdgeCollision(bodyA, bodyB, timeRemaining);
            CollisionInfo colBA = findVertexEdgeCollision(bodyB, bodyA, timeRemaining);

            if (colAB != null && colBA != null &&
                    Math.abs(colAB.collisionTime - colBA.collisionTime) <= COLLISION_EPSILON) {
                Vector2D relVel = bodyA.getVelocity().subtract(bodyB.getVelocity());
                double approachAB = relVel.dot(colAB.collisionNormal);
                double approachBA = relVel.scale(-1).dot(colBA.collisionNormal);
                if (approachAB <= approachBA) colBA = null; else colAB = null;
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

        return simultaneous;
    }

    private CollisionInfo findVertexEdgeCollision(RigidBody bodyA, RigidBody bodyB, double timeRemaining) {
        Vector2D[] verticesA = bodyA.getVertices();
        Vector2D[] verticesB = bodyB.getVertices();

        double earliestTime = Double.MAX_VALUE;
        Vector2D earliestNormal = null;
        List<Vector2D> contactVertices = new ArrayList<>();

        for (int v = 0; v < verticesA.length; v++) {
            Vector2D vertexA = verticesA[v];
            for (int e = 0; e < verticesB.length; e++) {
                CollisionInfo collision = findVertexEdgeCollisionTime(
                    bodyA, bodyB, vertexA,
                    verticesB[e], verticesB[(e + 1) % verticesB.length],
                    timeRemaining);

                if (collision == null) continue;

                if (collision.collisionTime < earliestTime - COLLISION_EPSILON) {
                    earliestTime = collision.collisionTime;
                    earliestNormal = collision.collisionNormal;
                    contactVertices.clear();
                    contactVertices.add(vertexA);
                } else if (collision.collisionTime <= earliestTime + COLLISION_EPSILON) {
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

        double sumX = 0, sumY = 0;
        for (Vector2D v : contactVertices) { sumX += v.getX(); sumY += v.getY(); }
        Vector2D avgContact = new Vector2D(sumX / contactVertices.size(), sumY / contactVertices.size());

        return new CollisionInfo(bodyA, bodyB, avgContact, earliestNormal, earliestTime);
    }

    private CollisionInfo findVertexEdgeCollisionTime(RigidBody bodyA, RigidBody bodyB,
                                                       Vector2D vertexBody,
                                                       Vector2D edgeStart, Vector2D edgeEnd,
                                                       double timeRemaining) {
        Vector2D vertexWorldA    = bodySpaceToWorldSpace(bodyA, vertexBody);
        Vector2D edgeStartWorldB = bodySpaceToWorldSpace(bodyB, edgeStart);
        Vector2D edgeEndWorldB   = bodySpaceToWorldSpace(bodyB, edgeEnd);

        Vector2D edgeVector    = edgeEndWorldB.subtract(edgeStartWorldB);
        double   edgeMagnitude = edgeVector.magnitude();
        if (edgeMagnitude < COLLISION_EPSILON) return null;

        Vector2D edgeNormal  = new Vector2D(edgeVector.getY(), -edgeVector.getX()).scale(1.0 / edgeMagnitude);
        Vector2D vertexVelA  = getPointVelocity(bodyA, vertexBody);
        Vector2D edgePointVelB = getPointVelocity(bodyB, edgeStart);
        Vector2D relativeVel = vertexVelA.subtract(edgePointVelB);
        double   normalVel   = relativeVel.dot(edgeNormal);

        Vector2D toVertex       = vertexWorldA.subtract(edgeStartWorldB);
        double   initialDistance = toVertex.dot(edgeNormal);

        if (initialDistance > COLLISION_EPSILON && normalVel >= -COLLISION_EPSILON) return null;
        if (Math.abs(normalVel) < COLLISION_EPSILON) return null;

        double timeToCollision = -initialDistance / normalVel;
        if (timeToCollision < COLLISION_EPSILON || timeToCollision > timeRemaining + COLLISION_EPSILON) return null;

        Vector2D vertexAtCollision     = vertexWorldA.add(vertexVelA.scale(timeToCollision));
        Vector2D edgePointVelEnd       = getPointVelocity(bodyB, edgeEnd);
        Vector2D edgeStartAtCollision  = edgeStartWorldB.add(edgePointVelB.scale(timeToCollision));
        Vector2D edgeEndAtCollision    = edgeEndWorldB.add(edgePointVelEnd.scale(timeToCollision));

        Vector2D edgeVecAtCollision = edgeEndAtCollision.subtract(edgeStartAtCollision);
        Vector2D toCollision        = vertexAtCollision.subtract(edgeStartAtCollision);
        double   edgeLenSq          = edgeVecAtCollision.dot(edgeVecAtCollision);
        if (edgeLenSq < COLLISION_EPSILON) return null;

        double t = toCollision.dot(edgeVecAtCollision) / edgeLenSq;
        if (t < -COLLISION_EPSILON || t > 1.0 + COLLISION_EPSILON) return null;

        return new CollisionInfo(bodyA, bodyB, vertexBody, edgeNormal, timeToCollision);
    }

    // ── Collision response ────────────────────────────────────────────────────

    private void handleCollision(RigidBody bodyA, RigidBody bodyB, Vector2D vertexA, Vector2D normal) {
        Vector2D velocityA = getPointVelocity(bodyA, vertexA);

        Vector2D[] verticesB     = bodyB.getVertices();
        Vector2D   collisionPointB = vertexA;
        double     minDist         = Double.MAX_VALUE;

        Vector2D vertexAWorld = bodySpaceToWorldSpace(bodyA, vertexA);
        for (int i = 0; i < verticesB.length; i++) {
            Vector2D edgeStart      = verticesB[i];
            Vector2D edgeEnd        = verticesB[(i + 1) % verticesB.length];
            Vector2D edgeStartWorld = bodySpaceToWorldSpace(bodyB, edgeStart);
            Vector2D edgeEndWorld   = bodySpaceToWorldSpace(bodyB, edgeEnd);
            Vector2D edgeVec        = edgeEndWorld.subtract(edgeStartWorld);
            double   edgeLenSq      = edgeVec.dot(edgeVec);

            if (edgeLenSq > COLLISION_EPSILON) {
                double   t            = vertexAWorld.subtract(edgeStartWorld).dot(edgeVec) / edgeLenSq;
                t = Math.max(0, Math.min(1, t));
                Vector2D closestPoint = edgeStartWorld.add(edgeVec.scale(t));
                double   dist         = vertexAWorld.subtract(closestPoint).magnitude();

                if (dist < minDist) {
                    minDist = dist;
                    Vector2D relPos = closestPoint.subtract(bodyB.getWorldPos());
                    collisionPointB = relPos.rotate(-bodyB.getWorldOrientation());
                }
            }
        }

        Vector2D velocityB        = getPointVelocity(bodyB, collisionPointB);
        Vector2D relativeVel      = velocityA.subtract(velocityB);
        double   relativeVelNormal = relativeVel.dot(normal);

        if (relativeVelNormal > 1e-6) return;

        double e = (Math.abs(relativeVelNormal) < RESTITUTION_VELOCITY_THRESHOLD)
                   ? 0.0
                   : (bodyA.getCoefficientOfRestitution() + bodyB.getCoefficientOfRestitution()) / 2.0;

        Vector2D r_A = bodySpaceToWorldSpace(bodyA, vertexA).subtract(bodyA.getWorldPos());
        Vector2D r_B = bodySpaceToWorldSpace(bodyB, collisionPointB).subtract(bodyB.getWorldPos());

        double invMa = bodyA.isStatic() ? 0.0 : 1.0 / bodyA.getMass();
        double invMb = bodyB.isStatic() ? 0.0 : 1.0 / bodyB.getMass();
        double invIa = bodyA.isStatic() ? 0.0 : 1.0 / bodyA.getMomentOfInertia();
        double invIb = bodyB.isStatic() ? 0.0 : 1.0 / bodyB.getMomentOfInertia();

        double r_a_cross_n  = r_A.getX() * normal.getY() - r_A.getY() * normal.getX();
        double r_b_cross_n  = r_B.getX() * normal.getY() - r_B.getY() * normal.getX();
        double denominator  = invMa + invMb + r_a_cross_n * r_a_cross_n * invIa
                                            + r_b_cross_n * r_b_cross_n * invIb;
        double j = -(1.0 + e) * relativeVelNormal / denominator;

        Vector2D impulse = normal.scale(j);
        if (!bodyA.isStatic()) {
            bodyA.setVelocity(bodyA.getVelocity().add(impulse.scale(invMa)));
            bodyA.setAngularVelocity(bodyA.getAngularVelocity() + r_a_cross_n * j * invIa);
        }
        if (!bodyB.isStatic()) {
            bodyB.setVelocity(bodyB.getVelocity().add(impulse.scale(-invMb)));
            bodyB.setAngularVelocity(bodyB.getAngularVelocity() - r_b_cross_n * j * invIb);
        }
    }

    // ── Friction ──────────────────────────────────────────────────────────────

    private void detectAndApplyFriction(List<RigidBody> bodies, double dt, List<int[]> candidates) {
        for (int[] pair : candidates) {
            RigidBody a = bodies.get(pair[0]), b = bodies.get(pair[1]);
            if (!boundingCirclesOverlap(a, b, CONTACT_SKIN)) continue;
            applyFrictionForPair(a, b, dt);
            applyFrictionForPair(b, a, dt);
        }
    }

    private void applyFrictionForPair(RigidBody bodyA, RigidBody bodyB, double dt) {
        if (bodyA.isStatic()) return;

        Vector2D[] vertsA     = bodyA.getVertices();
        Vector2D[] worldVertsA = bodyA.getWorldVertices();
        Vector2D[] worldVertsB = bodyB.getWorldVertices();

        List<Vector2D> contactBodyVerts = new ArrayList<>();
        Vector2D       contactNormal    = null;
        double         bestAbsDist      = Double.MAX_VALUE;

        for (int vi = 0; vi < vertsA.length; vi++) {
            Vector2D vBody  = vertsA[vi];
            Vector2D vWorld = worldVertsA[vi];

            for (int e = 0; e < worldVertsB.length; e++) {
                Vector2D e0w     = worldVertsB[e];
                Vector2D e1w     = worldVertsB[(e + 1) % worldVertsB.length];
                Vector2D edgeVec = e1w.subtract(e0w);
                double   edgeMag = edgeVec.magnitude();
                if (edgeMag < COLLISION_EPSILON) continue;

                Vector2D edgeNormal = new Vector2D(edgeVec.getY(), -edgeVec.getX()).scale(1.0 / edgeMag);
                double   dist       = vWorld.subtract(e0w).dot(edgeNormal);
                if (dist < -CONTACT_SKIN || dist > CONTACT_SKIN) continue;

                double along = vWorld.subtract(e0w).dot(edgeVec) / (edgeMag * edgeMag);
                if (along < 0 || along > 1) continue;

                contactBodyVerts.add(vBody);
                if (Math.abs(dist) < bestAbsDist) {
                    bestAbsDist  = Math.abs(dist);
                    contactNormal = edgeNormal;
                }
                break;
            }
        }

        if (contactBodyVerts.isEmpty() || contactNormal == null) return;

        double sx = 0, sy = 0;
        for (Vector2D v : contactBodyVerts) { sx += v.getX(); sy += v.getY(); }
        Vector2D avgContactA = new Vector2D(sx / contactBodyVerts.size(), sy / contactBodyVerts.size());

        Vector2D totalPermForce = new Vector2D(0, 0);
        for (Force f : bodyA.getAppliedForces()) totalPermForce = totalPermForce.add(f.getForceVector());
        double normalForce = -totalPermForce.dot(contactNormal);
        if (normalForce <= 0) return;

        Vector2D velA         = getPointVelocity(bodyA, avgContactA);
        Vector2D contactWorld = bodySpaceToWorldSpace(bodyA, avgContactA);
        Vector2D relToCenterB = contactWorld.subtract(bodyB.getWorldPos());
        Vector2D contactBBody = relToCenterB.rotate(-bodyB.getWorldOrientation());
        Vector2D velB         = getPointVelocity(bodyB, contactBBody);

        Vector2D relVel            = velA.subtract(velB);
        Vector2D tangentialRelVel  = relVel.subtract(contactNormal.scale(relVel.dot(contactNormal)));
        double   tangentSpeed      = tangentialRelVel.magnitude();
        if (tangentSpeed < 1e-4) return;

        Vector2D tangent = tangentialRelVel.scale(1.0 / tangentSpeed);
        double   mu      = (bodyA.getCoefficientOfKineticFriction() + bodyB.getCoefficientOfKineticFriction()) / 2.0;

        Vector2D r_A         = contactWorld.subtract(bodyA.getWorldPos());
        Vector2D r_B         = contactWorld.subtract(bodyB.getWorldPos());
        double   invMa       = bodyA.isStatic() ? 0.0 : 1.0 / bodyA.getMass();
        double   invMb       = bodyB.isStatic() ? 0.0 : 1.0 / bodyB.getMass();
        double   invIa       = bodyA.isStatic() ? 0.0 : 1.0 / bodyA.getMomentOfInertia();
        double   invIb       = bodyB.isStatic() ? 0.0 : 1.0 / bodyB.getMomentOfInertia();
        double   r_a_cross_t = r_A.getX() * tangent.getY() - r_A.getY() * tangent.getX();
        double   r_b_cross_t = r_B.getX() * tangent.getY() - r_B.getY() * tangent.getX();
        double   effMassDenom = invMa + invMb + r_a_cross_t * r_a_cross_t * invIa
                                              + r_b_cross_t * r_b_cross_t * invIb;
        double   mEff         = (effMassDenom > COLLISION_EPSILON) ? 1.0 / effMassDenom : 0.0;

        double   frictionMagnitude = Math.min(mu * normalForce, tangentSpeed * mEff / dt);
        Vector2D frictionForce     = tangent.scale(-frictionMagnitude);
        bodyA.applyTemporaryForce(new Force(frictionForce, avgContactA));
        if (!bodyB.isStatic()) bodyB.applyTemporaryForce(new Force(frictionForce.scale(-1.0), contactBBody));
    }

    // ── Overlap resolution ────────────────────────────────────────────────────

    private boolean checkAndResolveOverlap(List<RigidBody> bodies, List<int[]> candidates) {
        boolean anyResolved = false;
        for (int[] pair : candidates) {
            RigidBody a = bodies.get(pair[0]), b = bodies.get(pair[1]);
            if (resolveOverlapDirected(a, b)) anyResolved = true;
            if (resolveOverlapDirected(b, a)) anyResolved = true;
        }
        return anyResolved;
    }

    /**
     * Attempts to resolve penetration of body A into body B.
     * A is the body whose position is corrected; B may also receive a proportional correction.
     * Returns true if an overlap above OVERLAP_SLOP was found and resolved.
     */
    private boolean resolveOverlapDirected(RigidBody A, RigidBody B) {
        if (A.isStatic()) return false;
        if (A.isSleeping() && B.isSleeping()) return false;
        if (!boundingCirclesOverlap(A, B, 0)) return false;

        Vector2D[] vertsA = A.getVertices();
        Vector2D[] worldA = A.getWorldVertices();
        Vector2D[] worldB = B.getWorldVertices();

        double[] sat = satPenetration(worldA, worldB, A.getWorldPos(), B.getWorldPos());
        if (sat == null) return false;

        double depth = sat[0];
        if (depth < OVERLAP_SLOP) return false;
        Vector2D resolveNormal = new Vector2D(sat[1], sat[2]);

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

        double invMassA    = 1.0 / A.getMass();
        double invMassB    = B.isStatic() ? 0.0 : 1.0 / B.getMass();
        double totalInvMass = invMassA + invMassB;
        A.setWorldPos(A.getWorldPos().add(resolveNormal.scale(depth * invMassA / totalInvMass)));
        if (!B.isStatic()) B.setWorldPos(B.getWorldPos().subtract(resolveNormal.scale(depth * invMassB / totalInvMass)));

        Vector2D vel           = A.getVelocity();
        double   centerApproach = vel.dot(resolveNormal);
        Vector2D contactVel    = getPointVelocity(A, avgContact);
        double   contactApproach = contactVel.dot(resolveNormal);

        if (centerApproach < 0 && contactApproach >= 0) {
            A.setVelocity(vel.subtract(resolveNormal.scale(centerApproach)));
            contactVel = getPointVelocity(A, avgContact);
        }

        if (contactVel.dot(resolveNormal) < 0) handleCollision(A, B, avgContact, resolveNormal);

        return true;
    }

    private double[] satPenetration(Vector2D[] worldA, Vector2D[] worldB,
                                    Vector2D centerA, Vector2D centerB) {
        double minDepth = Double.MAX_VALUE;
        double bestNX = 0, bestNY = 0;

        for (int pass = 0; pass < 2; pass++) {
            Vector2D[] poly = (pass == 0) ? worldA : worldB;
            for (int i = 0; i < poly.length; i++) {
                Vector2D e0 = poly[i], e1 = poly[(i + 1) % poly.length];
                double ex = e1.getX() - e0.getX(), ey = e1.getY() - e0.getY();
                double mag = Math.sqrt(ex * ex + ey * ey);
                if (mag < COLLISION_EPSILON) continue;
                double nx = ey / mag, ny = -ex / mag;

                double minA = Double.MAX_VALUE, maxA = -Double.MAX_VALUE;
                for (Vector2D v : worldA) {
                    double p = v.getX() * nx + v.getY() * ny;
                    if (p < minA) minA = p; if (p > maxA) maxA = p;
                }
                double minB = Double.MAX_VALUE, maxB = -Double.MAX_VALUE;
                for (Vector2D v : worldB) {
                    double p = v.getX() * nx + v.getY() * ny;
                    if (p < minB) minB = p; if (p > maxB) maxB = p;
                }

                double overlap = Math.min(maxA, maxB) - Math.max(minA, minB);
                if (overlap <= 0) return null;

                if (overlap < minDepth) {
                    minDepth = overlap;
                    double dot  = (centerA.getX() - centerB.getX()) * nx
                                + (centerA.getY() - centerB.getY()) * ny;
                    double sign = (dot >= 0) ? 1.0 : -1.0;
                    bestNX = nx * sign; bestNY = ny * sign;
                }
            }
        }
        return new double[]{minDepth, bestNX, bestNY};
    }

    // ── Utilities ─────────────────────────────────────────────────────────────

    private Vector2D bodySpaceToWorldSpace(RigidBody body, Vector2D bodyPoint) {
        return bodyPoint.rotate(body.getWorldOrientation()).add(body.getWorldPos());
    }

    private Vector2D getPointVelocity(RigidBody body, Vector2D bodySpacePoint) {
        Vector2D worldPoint = bodySpaceToWorldSpace(body, bodySpacePoint);
        Vector2D toPoint    = worldPoint.subtract(body.getWorldPos());
        return body.getVelocity().add(new Vector2D(
                -body.getAngularVelocity() * toPoint.getY(),
                 body.getAngularVelocity() * toPoint.getX()));
    }

    // ── Spatial grid ──────────────────────────────────────────────────────────

    /**
     * Uniform spatial grid for O(n) candidate pair generation.
     *
     * Cell size = 2 × max bounding radius across all bodies, so every body spans
     * at most a 3×3 region of cells and bodies in non-adjacent cells are never
     * paired as candidates.
     *
     * The grid is rebuilt from scratch each physics step; cell lists and the pair
     * buffer are reused to avoid per-step allocation after the first build.
     */
    private static final class SpatialGrid {
        private double cellSize = 1.0;
        private double originX, originY;
        private int cols, rows;

        // Cell storage: flat array of lists, indexed row * cols + col.
        private List<Integer>[] cells = null;
        private int activeCells = 0; // number of cells in current build (cols * rows)

        // Pair output buffer — owned here, valid until next build().
        private final List<int[]> pairBuffer = new ArrayList<>();
        // Deduplication scratch — never cleared with fill(), only toggled entries are reset.
        private boolean[] seen = new boolean[0];

        @SuppressWarnings("unchecked")
        void build(List<RigidBody> bodies, double expansion) {
            int n = bodies.size();
            pairBuffer.clear();
            if (n == 0) { activeCells = 0; return; }

            // Single pass: compute world AABB and max bounding radius.
            double minX = Double.MAX_VALUE, maxX = -Double.MAX_VALUE;
            double minY = Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
            double maxR = 0;
            for (RigidBody b : bodies) {
                double r  = b.getBoundingRadius() + expansion;
                double bx = b.getWorldPos().getX(), by = b.getWorldPos().getY();
                if (r  > maxR)  maxR  = r;
                if (bx - r < minX) minX = bx - r;
                if (bx + r > maxX) maxX = bx + r;
                if (by - r < minY) minY = by - r;
                if (by + r > maxY) maxY = by + r;
            }

            cellSize = Math.max(maxR * 2.0, 1.0);
            originX  = minX;
            originY  = minY;
            cols = Math.max(1, (int) Math.ceil((maxX - minX) / cellSize));
            rows = Math.max(1, (int) Math.ceil((maxY - minY) / cellSize));
            activeCells = cols * rows;

            // Grow / initialise cell array if needed; clear active cells.
            if (cells == null || cells.length < activeCells) {
                cells = new List[activeCells];
                for (int i = 0; i < activeCells; i++) cells[i] = new ArrayList<>();
            } else {
                for (int i = 0; i < activeCells; i++) cells[i].clear();
            }

            // Insert each body into every cell its expanded circle overlaps.
            for (int idx = 0; idx < n; idx++) {
                RigidBody b = bodies.get(idx);
                double r  = b.getBoundingRadius() + expansion;
                double bx = b.getWorldPos().getX(), by = b.getWorldPos().getY();

                int c0 = Math.max(0,        (int) ((bx - r - originX) / cellSize));
                int c1 = Math.min(cols - 1, (int) ((bx + r - originX) / cellSize));
                int r0 = Math.max(0,        (int) ((by - r - originY) / cellSize));
                int r1 = Math.min(rows - 1, (int) ((by + r - originY) / cellSize));

                for (int row = r0; row <= r1; row++)
                    for (int col = c0; col <= c1; col++)
                        cells[row * cols + col].add(idx);
            }
        }

        /**
         * Returns unique unordered candidate pairs (pair[0] < pair[1]) that share
         * at least one grid cell. Valid until the next call to build().
         */
        List<int[]> candidatePairs(int n) {
            if (n == 0 || activeCells == 0) return pairBuffer;

            // Grow deduplication array if needed (never shrinks — minor memory trade-off).
            if (seen.length < n * n) seen = new boolean[n * n];

            for (int c = 0; c < activeCells; c++) {
                List<Integer> cell = cells[c];
                int sz = cell.size();
                for (int a = 0; a < sz; a++) {
                    for (int b = a + 1; b < sz; b++) {
                        int i = cell.get(a), j = cell.get(b);
                        if (i > j) { int tmp = i; i = j; j = tmp; }
                        int key = i * n + j;
                        if (!seen[key]) {
                            seen[key] = true;
                            pairBuffer.add(new int[]{i, j});
                        }
                    }
                }
            }

            // Reset only the entries we wrote — O(pairs) not O(n²).
            for (int[] pair : pairBuffer) seen[pair[0] * n + pair[1]] = false;

            return pairBuffer;
        }
    }

    // ── Collision info record ─────────────────────────────────────────────────

    private static class CollisionInfo {
        RigidBody bodyA, bodyB;
        Vector2D  vertexA, collisionNormal;
        double    collisionTime;

        CollisionInfo(RigidBody bodyA, RigidBody bodyB,
                      Vector2D vertexA, Vector2D collisionNormal, double collisionTime) {
            this.bodyA = bodyA; this.bodyB = bodyB;
            this.vertexA = vertexA; this.collisionNormal = collisionNormal;
            this.collisionTime = collisionTime;
        }
    }
}
