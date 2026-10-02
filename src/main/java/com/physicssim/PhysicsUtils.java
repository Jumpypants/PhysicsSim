package com.physicssim;

import java.util.List;

public class PhysicsUtils {
    private PhysicsUtils() {}

    /**
     * Returns true if any vertex of {@code body} is resting on an upward-facing surface
     * of another body within {@code skinDistance} units.
     */
    public static boolean isGrounded(RigidBody body, List<RigidBody> others, double skinDistance) {
        Vector2D[] verts = body.getVertices();
        for (RigidBody other : others) {
            if (other == body) continue;
            Vector2D[] otherVerts = other.getVertices();
            for (Vector2D vBody : verts) {
                Vector2D vWorld = body.toWorldSpace(vBody);
                for (int e = 0; e < otherVerts.length; e++) {
                    Vector2D e0 = other.toWorldSpace(otherVerts[e]);
                    Vector2D e1 = other.toWorldSpace(otherVerts[(e + 1) % otherVerts.length]);
                    Vector2D edgeVec = e1.subtract(e0);
                    double edgeMag = edgeVec.magnitude();
                    if (edgeMag < 1e-6) continue;

                    Vector2D normal = new Vector2D(edgeVec.getY(), -edgeVec.getX()).scale(1.0 / edgeMag);
                    if (normal.getY() < 0.5) continue;

                    double dist = vWorld.subtract(e0).dot(normal);
                    if (dist < -skinDistance || dist > skinDistance) continue;

                    double along = vWorld.subtract(e0).dot(edgeVec) / (edgeMag * edgeMag);
                    if (along < 0 || along > 1) continue;

                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Returns the first body within {@code skinDistance} of any vertex of {@code body},
     * regardless of edge normal direction. Returns null if no contact is found.
     * Used by the kick attack to detect the first body hit and distinguish static
     * surfaces (walls/floor) from dynamic bodies (players).
     */
    public static RigidBody getContactBody(RigidBody body, List<RigidBody> others, double skinDistance) {
        Vector2D[] worldVerts = body.getWorldVertices();
        for (RigidBody other : others) {
            if (other == body) continue;
            Vector2D[] otherWorld = other.getWorldVertices();
            int n = otherWorld.length;
            for (Vector2D v : worldVerts) {
                for (int e = 0; e < n; e++) {
                    Vector2D e0 = otherWorld[e], e1 = otherWorld[(e + 1) % n];
                    double ex = e1.getX() - e0.getX(), ey = e1.getY() - e0.getY();
                    double edgeLenSq = ex * ex + ey * ey;
                    if (edgeLenSq < 1e-12) continue;
                    double tx = v.getX() - e0.getX(), ty = v.getY() - e0.getY();
                    double t = (tx * ex + ty * ey) / edgeLenSq;
                    if (t < 0 || t > 1) continue;
                    double dist = Math.abs(tx * ey - ty * ex) / Math.sqrt(edgeLenSq);
                    if (dist <= skinDistance) return other;
                }
            }
        }
        return null;
    }

    /**
     * Returns the outward-facing normal of a wall surface the body is touching,
     * or null if no wall contact is found. A wall is any edge whose normal is
     * predominantly horizontal (|normalX| > 0.7). The returned normal points away
     * from the wall toward the body — positive X means the wall is to the left,
     * negative X means the wall is to the right.
     */
    public static Vector2D getWallNormal(RigidBody body, List<RigidBody> others, double skinDistance) {
        Vector2D[] verts = body.getVertices();
        for (RigidBody other : others) {
            if (other == body) continue;
            Vector2D[] otherVerts = other.getVertices();
            for (Vector2D vBody : verts) {
                Vector2D vWorld = body.toWorldSpace(vBody);
                for (int e = 0; e < otherVerts.length; e++) {
                    Vector2D e0 = other.toWorldSpace(otherVerts[e]);
                    Vector2D e1 = other.toWorldSpace(otherVerts[(e + 1) % otherVerts.length]);
                    Vector2D edgeVec = e1.subtract(e0);
                    double edgeMag = edgeVec.magnitude();
                    if (edgeMag < 1e-6) continue;

                    Vector2D normal = new Vector2D(edgeVec.getY(), -edgeVec.getX()).scale(1.0 / edgeMag);
                    if (Math.abs(normal.getX()) < 0.7) continue;

                    double dist = vWorld.subtract(e0).dot(normal);
                    if (dist < -skinDistance || dist > skinDistance) continue;

                    double along = vWorld.subtract(e0).dot(edgeVec) / (edgeMag * edgeMag);
                    if (along < 0 || along > 1) continue;

                    return normal;
                }
            }
        }
        return null;
    }
}
