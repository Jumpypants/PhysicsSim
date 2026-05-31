package com.physicssim.demo;

import com.physicssim.PhysicsSimulator;
import com.physicssim.RigidBody;
import com.physicssim.Vector2D;

import java.util.ArrayList;
import java.util.List;

/**
 * Headless test for collision detection.
 */
public class CollisionTest {
    public static void main(String[] args) {
        System.out.println("Starting collision test...");

        List<RigidBody> bodies = new ArrayList<>();

        // Create two squares
        Vector2D[] squareVertices = new Vector2D[]{
            new Vector2D(-0.5, -0.5),  // Bottom-left
            new Vector2D(0.5, -0.5),   // Bottom-right
            new Vector2D(0.5, 0.5),    // Top-right
            new Vector2D(-0.5, 0.5)    // Top-left
        };

        // Square 1: bottom position
        RigidBody square1 = new RigidBody(squareVertices, 1.0, new Vector2D(0, -2), 0);
        bodies.add(square1);

        // Square 2: top position, falling down
        RigidBody square2 = new RigidBody(squareVertices, 1.0, new Vector2D(0, 0), 0);
        square2.setVelocity(new Vector2D(0, -1.0)); // Moving down
        bodies.add(square2);

        System.out.println("Square 1 at: " + square1.getWorldPos().getX() + ", " + square1.getWorldPos().getY());
        System.out.println("Square 2 at: " + square2.getWorldPos().getX() + ", " + square2.getWorldPos().getY());
        System.out.println("Square 1 velocity: " + square1.getVelocity().getX() + ", " + square1.getVelocity().getY());
        System.out.println("Square 2 velocity: " + square2.getVelocity().getX() + ", " + square2.getVelocity().getY());

        PhysicsSimulator simulator = new PhysicsSimulator();

        // Simulate for enough time to get a collision
        long startTime = System.currentTimeMillis();
        for (int i = 0; i < 2000; i++) { // Extended to 2 seconds
            double vy1_before = square1.getVelocity().getY();
            double vy2_before = square2.getVelocity().getY();

            simulator.simulateTimeStep(bodies, 0.001); // 1ms steps

            double vy1_after = square1.getVelocity().getY();
            double vy2_after = square2.getVelocity().getY();

            // Check if velocity changed (collision response)
            if (Math.abs(vy1_after - vy1_before) > 0.01 || Math.abs(vy2_after - vy2_before) > 0.01) {
                System.out.println("VELOCITY CHANGE at step " + i + "!");
                System.out.println("  Square 1 VY: " + vy1_before + " -> " + vy1_after);
                System.out.println("  Square 2 VY: " + vy2_before + " -> " + vy2_after);
            }

            double y1 = square1.getWorldPos().getY();
            double y2 = square2.getWorldPos().getY();
            if (i % 100 == 0) {
                System.out.println("Step " + i + ": Square1 Y=" + String.format("%.3f", y1) +
                                 ", Square2 Y=" + String.format("%.3f", y2) +
                                 ", Vel1=" + String.format("%.3f", square1.getVelocity().getY()));
            }

            // Check if collision happened (squares getting close and velocities changed dramatically)
            double dist = Math.abs(y2 - y1);
            if (dist < 1.0 && i > 10) { // Squares actually overlapping (1 unit is the size)
                System.out.println("OVERLAP DETECTED! Squares distance < 1.0!");
                System.out.println("Final state at step " + i + ":");
                System.out.println("  Square 1: Y=" + y1 + ", VY=" + square1.getVelocity().getY());
                System.out.println("  Square 2: Y=" + y2 + ", VY=" + square2.getVelocity().getY());
                System.out.println("  Distance: " + dist);
                long elapsed = System.currentTimeMillis() - startTime;
                System.out.println("Time elapsed: " + elapsed + "ms");
                System.exit(0);
            }
        }

        System.out.println("No collision detected after 2000 steps");
        System.out.println("Final state:");
        System.out.println("  Square 1: Y=" + square1.getWorldPos().getY() + ", VY=" + square1.getVelocity().getY());
        System.out.println("  Square 2: Y=" + square2.getWorldPos().getY() + ", VY=" + square2.getVelocity().getY());
    }
}
