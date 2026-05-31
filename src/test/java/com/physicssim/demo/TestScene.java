package com.physicssim.demo;

import com.physicssim.Force;
import com.physicssim.PhysicsSimulator;
import com.physicssim.RigidBody;
import com.physicssim.Vector2D;

import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;
import java.util.List;

/**
 * A test scene that displays rigid bodies using TestingRenderer.
 * Runs at a specified FPS to render the scene and a specified steps per second for physics.
 */
public class TestScene extends JPanel {
    private final List<RigidBody> rigidBodies = new ArrayList<>();
    private final TestingRenderer renderer = new TestingRenderer();
    private final PhysicsSimulator physicsSimulator = new PhysicsSimulator();

    private static final int TARGET_FPS = 60;
    private static final double FRAME_TIME = 1.0 / TARGET_FPS; // Time per frame in seconds

    private static final int PHYSICS_STEPS_PER_SECOND = 240; // Increased for better collision detection
    private static final double PHYSICS_STEP_TIME = 1.0 / PHYSICS_STEPS_PER_SECOND; // Time per physics step

    private static final int WINDOW_WIDTH = 800;
    private static final int WINDOW_HEIGHT = 600;
    private static final double PIXELS_PER_UNIT = 50.0; // Scale factor

    private static final double GRAVITY = 9.8; // Gravitational acceleration in m/s^2

    public TestScene() {
        setupScene();
        setPreferredSize(new Dimension(WINDOW_WIDTH, WINDOW_HEIGHT));
        setBackground(Color.WHITE);
    }

    /**
     * Sets up the initial scene with rigid bodies.
     */
    private void setupScene() {
        // Immovable floor spanning the visible width
        Vector2D[] floorVertices = new Vector2D[]{
            new Vector2D(-9, -0.3),
            new Vector2D( 9, -0.3),
            new Vector2D( 9,  0.3),
            new Vector2D(-9,  0.3)
        };
        rigidBodies.add(RigidBody.createStatic(floorVertices, new Vector2D(0, -5), 0));

        // Immovable side walls
        Vector2D[] wallVertices = new Vector2D[]{
            new Vector2D(-0.3, -7),
            new Vector2D( 0.3, -7),
            new Vector2D( 0.3,  7),
            new Vector2D(-0.3,  7)
        };
        rigidBodies.add(RigidBody.createStatic(wallVertices, new Vector2D(-8.3, 0), 0)); // left wall
        rigidBodies.add(RigidBody.createStatic(wallVertices, new Vector2D( 8.3, 0), 0)); // right wall

        // Heavy central hexagon — the "bumper" everything crashes into
        Vector2D[] hexVertices = regularPolygon(6, 0.9);
        RigidBody hex = new RigidBody(hexVertices, 6.0, new Vector2D(0, 0), 0);
        addGravity(hex, 6.0);
        rigidBodies.add(hex);

        // Thin plank sliding in fast from the right
        Vector2D[] plankVertices = {
            new Vector2D(-1.2, -0.2), new Vector2D(1.2, -0.2),
            new Vector2D( 1.2,  0.2), new Vector2D(-1.2, 0.2)
        };
        RigidBody plank = new RigidBody(plankVertices, 1.5, new Vector2D(6, -1), Math.PI / 10);
        plank.setVelocity(new Vector2D(-5.0, 0.5));
        addGravity(plank, 1.5);
        rigidBodies.add(plank);

        // Tall domino dropping near the right side
        Vector2D[] dominoVertices = {
            new Vector2D(-0.2, -0.7), new Vector2D(0.2, -0.7),
            new Vector2D( 0.2,  0.7), new Vector2D(-0.2, 0.7)
        };
        RigidBody domino = new RigidBody(dominoVertices, 0.9, new Vector2D(5.5, 3), 0.15);
        domino.setVelocity(new Vector2D(-1.0, -0.5));
        addGravity(domino, 0.9);
        rigidBodies.add(domino);

        // Trapezoid (wide base, narrow top) falling from the upper-left
        Vector2D[] trapVertices = {
            new Vector2D(-0.9, -0.4), new Vector2D( 0.9, -0.4),
            new Vector2D( 0.4,  0.4), new Vector2D(-0.4,  0.4)
        };
        RigidBody trap = new RigidBody(trapVertices, 1.4, new Vector2D(-2, 5), -0.2);
        trap.setVelocity(new Vector2D(1.5, -3.0));
        trap.setAngularVelocity(0.9);
        addGravity(trap, 1.4);
        rigidBodies.add(trap);

        // 5-pointed star (concave) flying in from the upper-left with spin
        Vector2D[] starVertices = star(0.7, 0.28, 5);
        RigidBody starBody = new RigidBody(starVertices, 1.0, new Vector2D(-6, 4), 0.0);
        starBody.setVelocity(new Vector2D(3.0, -1.5));
        starBody.setAngularVelocity(-1.2);
        addGravity(starBody, 1.0);
        rigidBodies.add(starBody);

        // Right-pointing arrow (concave V-notch at the back) shot from the lower-right
        Vector2D[] arrowVertices = {
            new Vector2D( 0.9,  0.0),  // tip
            new Vector2D( 0.1,  0.55), // upper wing
            new Vector2D( 0.1,  0.22), // inner upper notch
            new Vector2D(-0.9,  0.22), // back upper
            new Vector2D(-0.9, -0.22), // back lower
            new Vector2D( 0.1, -0.22), // inner lower notch
            new Vector2D( 0.1, -0.55)  // lower wing
        };
        RigidBody arrow = new RigidBody(arrowVertices, 1.1, new Vector2D(5, -3), 0.0);
        arrow.setVelocity(new Vector2D(-4.0, 4.0));
        arrow.setAngularVelocity(1.8);
        addGravity(arrow, 1.1);
        rigidBodies.add(arrow);
    }

    /**
     * Returns vertices for a star polygon with the given point count, wound CCW.
     * Alternates between outerRadius (tips) and innerRadius (notches).
     */
    private static Vector2D[] star(double outerRadius, double innerRadius, int points) {
        int n = points * 2;
        Vector2D[] verts = new Vector2D[n];
        for (int i = 0; i < n; i++) {
            // Start from the top (π/2) and step CCW in equal angular increments
            double angle = Math.PI / 2.0 + 2.0 * Math.PI * i / n;
            double r = (i % 2 == 0) ? outerRadius : innerRadius;
            verts[i] = new Vector2D(r * Math.cos(angle), r * Math.sin(angle));
        }
        return verts;
    }

    /** Returns vertices for a regular n-gon with the given radius, wound CCW. */
    private static Vector2D[] regularPolygon(int sides, double radius) {
        Vector2D[] verts = new Vector2D[sides];
        for (int i = 0; i < sides; i++) {
            double angle = 2 * Math.PI * i / sides;
            verts[i] = new Vector2D(radius * Math.cos(angle), radius * Math.sin(angle));
        }
        return verts;
    }

    private void addGravity(RigidBody body, double mass) {
        body.applyForce(new Force(new Vector2D(0, -mass * GRAVITY), new Vector2D(0, 0)));
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;

        // Enable anti-aliasing for smoother rendering
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // Get the center of the window
        double screenCenterX = WINDOW_WIDTH / 2.0;
        double screenCenterY = WINDOW_HEIGHT / 2.0;

        // Draw all rigid bodies
        renderer.drawRigidBodies(g2d, rigidBodies, screenCenterX, screenCenterY, PIXELS_PER_UNIT);
    }

    /**
     * Starts the rendering loop with physics simulation.
     */
    public void startRenderLoop() {
        Thread renderThread = new Thread(() -> {
            long lastFrameTime = System.nanoTime();
            long frameTimeNanos = (long) (FRAME_TIME * 1_000_000_000);

            while (true) {
                long currentTime = System.nanoTime();

                // Render frame
                long renderDeltaNanos = currentTime - lastFrameTime;
                if (renderDeltaNanos >= frameTimeNanos) {
                    // Update physics for this frame
                    updatePhysics(FRAME_TIME);
                    repaint();
                    lastFrameTime = currentTime;
                }

                // Sleep to avoid excessive CPU usage
                try {
                    long sleepTime = frameTimeNanos - (System.nanoTime() - lastFrameTime);
                    if (sleepTime > 0) {
                        Thread.sleep(sleepTime / 1_000_000); // Convert to milliseconds
                    }
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        });

        renderThread.setDaemon(true);
        renderThread.start();
    }

    /**
     * Updates the physics simulation by one frame time, subdividing into smaller physics steps.
     *
     * @param deltaTime The time step in seconds (one frame time)
     */
    private void updatePhysics(double deltaTime) {
        double timeAccumulated = 0;
        while (timeAccumulated < deltaTime) {
            double stepSize = Math.min(PHYSICS_STEP_TIME, deltaTime - timeAccumulated);
            physicsSimulator.simulateTimeStep(rigidBodies, stepSize);
            timeAccumulated += stepSize;
        }
    }

    /**
     * Main method to run the test scene.
     */
    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> {
            JFrame frame = new JFrame("Physics Simulation - Test Scene");
            TestScene scene = new TestScene();

            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            frame.add(scene);
            frame.pack();
            frame.setLocationRelativeTo(null);
            frame.setVisible(true);

            // Start the rendering loop
            scene.startRenderLoop();
        });
    }
}
