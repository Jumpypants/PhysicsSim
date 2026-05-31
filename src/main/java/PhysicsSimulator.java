import java.util.List;

/**
 * PhysicsSimulator handles collision detection and response for rigid bodies.
 * Uses vertex-to-edge collision detection and impulse-based collision response.
 */
public class PhysicsSimulator {
    private static final double COLLISION_EPSILON = 1e-6; // Small epsilon to avoid duplicate collisions
    private static final int MAX_COLLISIONS_PER_FRAME = 100; // Prevent infinite loops

    /**
     * Simulates a single time step, handling collisions.
     * Subdivides the time step into smaller intervals whenever a collision occurs.
     *
     * @param rigidBodies List of rigid bodies to simulate
     * @param timeRemaining Time step to simulate (in seconds)
     */
    public void simulateTimeStep(List<RigidBody> rigidBodies, double timeRemaining) {
        int collisionCount = 0;

        while (timeRemaining > COLLISION_EPSILON && collisionCount < MAX_COLLISIONS_PER_FRAME) {
            // Find the next collision within the entire remaining time
            CollisionInfo nextCollision = findNextCollision(rigidBodies, timeRemaining);

            if (nextCollision == null) {
                // No more collisions; step all bodies to the end
                for (RigidBody body : rigidBodies) {
                    body.stepTime(timeRemaining);
                }

                // After stepping, check for any penetrating pairs that were missed by CCD and resolve them
                if (checkAndResolveOverlap(rigidBodies)) {
                    // If we resolved an overlap, continue the loop to allow additional collisions within the same frame
                    collisionCount++;
                    // We consumed the whole remaining time (we stepped by timeRemaining)
                    timeRemaining = 0;
                    continue;
                }

                break;
            }

            // Step all bodies to the collision time
            for (RigidBody body : rigidBodies) {
                body.stepTime(nextCollision.collisionTime);
            }

            // Handle the collision
            handleCollision(nextCollision.bodyA, nextCollision.bodyB,
                          nextCollision.vertexA, nextCollision.collisionNormal);

            // Update time remaining
            timeRemaining -= nextCollision.collisionTime;
            collisionCount++;
        }
    }

    /**
     * Finds the next collision that will occur within the time remaining.
     * Tests all vertex-to-edge pairs.
     *
     * @param rigidBodies List of rigid bodies
     * @param timeRemaining Maximum time to look ahead
     * @return CollisionInfo of the next collision, or null if no collision
     */
    private CollisionInfo findNextCollision(List<RigidBody> rigidBodies, double timeRemaining) {
        CollisionInfo earliestCollision = null;

        // Check all pairs of bodies
        for (int i = 0; i < rigidBodies.size(); i++) {
            for (int j = i + 1; j < rigidBodies.size(); j++) {
                RigidBody bodyA = rigidBodies.get(i);
                RigidBody bodyB = rigidBodies.get(j);

                // Check all vertices of bodyA against all edges of bodyB
                CollisionInfo collision = findVertexEdgeCollision(bodyA, bodyB, timeRemaining);
                if (collision != null && (earliestCollision == null || collision.collisionTime < earliestCollision.collisionTime)) {
                    earliestCollision = collision;
                }

                // Check all vertices of bodyB against all edges of bodyA
                collision = findVertexEdgeCollision(bodyB, bodyA, timeRemaining);
                if (collision != null && (earliestCollision == null || collision.collisionTime < earliestCollision.collisionTime)) {
                    earliestCollision = collision;
                }
            }
        }

        return earliestCollision;
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
        CollisionInfo earliestCollision = null;

        for (int v = 0; v < verticesA.length; v++) {
            Vector2D vertexA = verticesA[v];

            for (int e = 0; e < verticesB.length; e++) {
                Vector2D edgeStart = verticesB[e];
                Vector2D edgeEnd = verticesB[(e + 1) % verticesB.length];

                CollisionInfo collision = findVertexEdgeCollisionTime(
                    bodyA, bodyB, vertexA, edgeStart, edgeEnd, timeRemaining
                );

                if (collision != null && (earliestCollision == null || collision.collisionTime < earliestCollision.collisionTime)) {
                    earliestCollision = collision;
                }
            }
        }

        return earliestCollision;
    }

    /**
     * Calculates the time at which a vertex (from bodyA) collides with an edge (from bodyB).
     * Uses relative motion to solve for collision time.
     *
     * @param bodyA Body containing the vertex
     * @param bodyB Body containing the edge
     * @param vertexIndex Index of the vertex in bodyA
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

        // Average coefficient of restitution
        double e = (bodyA.getCoefficientOfRestitution() + bodyB.getCoefficientOfRestitution()) / 2.0;

        // Get position vectors from body centers to collision point (in world-space)
        Vector2D r_A = bodySpaceToWorldSpace(bodyA, vertexA).subtract(bodyA.getWorldPos());
        Vector2D r_B = bodySpaceToWorldSpace(bodyB, collisionPointB).subtract(bodyB.getWorldPos());

        // Calculate impulse magnitude using the impulse-momentum equations
        // j = -(1 + e) * (v_rel · n) / (1/m_a + 1/m_b + (r_a × n)^2 / I_a + (r_b × n)^2 / I_b)

        double m_a = bodyA.getMass();
        double m_b = bodyB.getMass();
        double I_a = bodyA.getMomentOfInertia();
        double I_b = bodyB.getMomentOfInertia();

        // Cross products (in 2D, a × b gives a scalar)
        double r_a_cross_n = r_A.getX() * normal.getY() - r_A.getY() * normal.getX();
        double r_b_cross_n = r_B.getX() * normal.getY() - r_B.getY() * normal.getX();

        double denominator = (1.0 / m_a) + (1.0 / m_b) +
                           (r_a_cross_n * r_a_cross_n) / I_a +
                           (r_b_cross_n * r_b_cross_n) / I_b;

        double j = -(1.0 + e) * relativeVelNormal / denominator;

        // Apply impulse: J = j * n
        Vector2D impulse = normal.scale(j);

        // Update velocities
        Vector2D newVelA = bodyA.getVelocity().add(impulse.scale(1.0 / m_a));
        Vector2D newVelB = bodyB.getVelocity().add(impulse.scale(-1.0 / m_b));

        bodyA.setVelocity(newVelA);
        bodyB.setVelocity(newVelB);

        // Update angular velocities
        double newAngularVelA = bodyA.getAngularVelocity() + r_a_cross_n * j / I_a;
        double newAngularVelB = bodyB.getAngularVelocity() - r_b_cross_n * j / I_b;

        bodyA.setAngularVelocity(newAngularVelA);
        bodyB.setAngularVelocity(newAngularVelB);
    }

    // Check for overlapping polygons after stepping and resolve first found penetration via an impulse
    private boolean checkAndResolveOverlap(List<RigidBody> bodies) {
        for (int i = 0; i < bodies.size(); i++) {
            RigidBody A = bodies.get(i);
            Vector2D[] vertsA = A.getVertices();
            for (int j = 0; j < bodies.size(); j++) {
                if (i == j) continue;
                RigidBody B = bodies.get(j);
                Vector2D[] vertsB = B.getVertices();

                // Build world-space vertex arrays for B once
                Vector2D[] worldB = new Vector2D[vertsB.length];
                for (int k = 0; k < vertsB.length; k++) worldB[k] = bodySpaceToWorldSpace(B, vertsB[k]);

                // For each vertex of A, check if it's inside polygon B
                for (int va = 0; va < vertsA.length; va++) {
                    Vector2D vBody = vertsA[va];
                    Vector2D vWorld = bodySpaceToWorldSpace(A, vBody);
                    if (pointInPolygon(vWorld, worldB)) {
                        System.out.println("PENETRATION: A vertex isclosed in B");
                        // Find closest edge on B to this point and compute normal from edge to point
                        int closestEdgeIdx = -1;
                        double bestDistSq = Double.MAX_VALUE;
                        Vector2D bestNormal = null;
                        for (int eb = 0; eb < worldB.length; eb++) {
                            Vector2D e0 = worldB[eb];
                            Vector2D e1 = worldB[(eb + 1) % worldB.length];
                            // project vWorld onto edge segment
                            Vector2D edge = e1.subtract(e0);
                            double edgeLenSq = edge.dot(edge);
                            double t = 0;
                            if (edgeLenSq > 0) {
                                t = vWorld.subtract(e0).dot(edge) / edgeLenSq;
                                t = Math.max(0, Math.min(1, t));
                            }
                            Vector2D proj = new Vector2D(e0.getX() + t * edge.getX(), e0.getY() + t * edge.getY());
                            Vector2D diff = vWorld.subtract(proj);
                            double distSq = diff.dot(diff);
                            if (distSq < bestDistSq) {
                                bestDistSq = distSq;
                                closestEdgeIdx = eb;
                                // normal is from edge to point
                                if (distSq > 0) {
                                    bestNormal = diff.scale(1.0 / Math.sqrt(distSq));
                                } else {
                                    // Degenerate: fallback to edge normal
                                    bestNormal = new Vector2D(-edge.getY(), edge.getX()).normalize();
                                }
                            }
                        }

                        if (closestEdgeIdx != -1 && bestNormal != null) {
                            // Resolve by applying an impulse at the penetrating vertex
                            // Get the velocity of the penetrating vertex along the normal
                            Vector2D vertexVel = getPointVelocity(A, vBody);
                            double velAlongNormal = vertexVel.dot(bestNormal);

                            // Only apply impulse if the vertex is moving into body B, or if it's barely separating/parallel
                            if (velAlongNormal < 0.1) { // Allow small positive velocity due to numerical errors
                                handleCollision(A, B, vBody, bestNormal);
                            } else {
                                // If clearly moving away, just separate the bodies
                                // Push A away from B
                                A.setWorldPos(A.getWorldPos().add(bestNormal.scale(0.01)));
                            }
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }

    // Ray-casting point-in-polygon test
    private boolean pointInPolygon(Vector2D pt, Vector2D[] poly) {
        boolean inside = false;
        for (int i = 0, j = poly.length - 1; i < poly.length; j = i++) {
            double xi = poly[i].getX(), yi = poly[i].getY();
            double xj = poly[j].getX(), yj = poly[j].getY();
            boolean intersect = ((yi > pt.getY()) != (yj > pt.getY())) &&
                (pt.getX() < (xj - xi) * (pt.getY() - yi) / (yj - yi + 1e-12) + xi);
            if (intersect) inside = !inside;
        }
        return inside;
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










