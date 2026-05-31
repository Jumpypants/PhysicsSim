package com.physicssim.demo;

import com.physicssim.Force;
import com.physicssim.PhysicsSimulator;
import com.physicssim.RigidBody;
import com.physicssim.Vector2D;

import java.util.ArrayList;
import java.util.List;

/**
 * Headless test to debug collision detection without GUI.
 */
public class HeadlessTest {
    private static final double GRAVITY = 9.8;
    private static final double FRAME_TIME = 1.0 / 60.0;

    public static void main(String[] args) {
        // Create two squares
        Vector2D[] squareVertices = new Vector2D[]{
            new Vector2D(-0.5, -0.5),  // Bottom-left
            new Vector2D(0.5, -0.5),   // Bottom-right
            new Vector2D(0.5, 0.5),    // Top-right
            new Vector2D(-0.5, 0.5)    // Top-left
        };

        double mass = 1.0;

        // Square 1: at origin with gravity
        Vector2D pos1 = new Vector2D(0, -1);
        RigidBody square1 = new RigidBody(squareVertices, mass, pos1, 0.0);
        Vector2D forceVec = new Vector2D(0, -mass * GRAVITY);
        square1.applyForce(new Force(forceVec, new Vector2D(0, 0)));

        // Square 2: above square 1, moving downward
        Vector2D pos2 = new Vector2D(0, 2);
        RigidBody square2 = new RigidBody(squareVertices, mass, pos2, 0.0);
        square2.setVelocity(new Vector2D(0, -10.0));  // Much faster to trigger collision within frame
        square2.applyForce(new Force(forceVec, new Vector2D(0, 0)));

        List<RigidBody> bodies = new ArrayList<>();
        bodies.add(square1);
        bodies.add(square2);

        PhysicsSimulator sim = new PhysicsSimulator();

        System.out.println("Initial state:");
        System.out.println("Square 1 pos: " + square1.getWorldPos());
        System.out.println("Square 2 pos: " + square2.getWorldPos());
        System.out.println("Square 2 vel: " + square2.getVelocity());

        // Run for a few frames
        for (int frame = 0; frame < 30; frame++) {
            System.out.println("\n--- Frame " + frame + " ---");
            System.out.println("Before step:");
            System.out.println("Square 1 pos: (" + square1.getWorldPos().getX() + ", " + square1.getWorldPos().getY() + ") vel: (" + square1.getVelocity().getX() + ", " + square1.getVelocity().getY() + ")");
            System.out.println("Square 2 pos: (" + square2.getWorldPos().getX() + ", " + square2.getWorldPos().getY() + ") vel: (" + square2.getVelocity().getX() + ", " + square2.getVelocity().getY() + ")");

            sim.simulateTimeStep(bodies, FRAME_TIME);

            System.out.println("After step:");
            System.out.println("Square 1 pos: (" + square1.getWorldPos().getX() + ", " + square1.getWorldPos().getY() + ") vel: (" + square1.getVelocity().getX() + ", " + square1.getVelocity().getY() + ")");
            System.out.println("Square 2 pos: (" + square2.getWorldPos().getX() + ", " + square2.getWorldPos().getY() + ") vel: (" + square2.getVelocity().getX() + ", " + square2.getVelocity().getY() + ")");

            double dist = square2.getWorldPos().getY() - square1.getWorldPos().getY();
            System.out.println("Distance between centers: " + dist);

            // Calculate edge positions
            double square1Top = square1.getWorldPos().getY() + 0.5;
            double square2Bottom = square2.getWorldPos().getY() - 0.5;
            double edgeGap = square2Bottom - square1Top;
            System.out.println("Square 1 top: " + square1Top + ", Square 2 bottom: " + square2Bottom + ", Gap: " + edgeGap);
        }
    }
}
