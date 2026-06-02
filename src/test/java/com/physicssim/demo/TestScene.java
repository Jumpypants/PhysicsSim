package com.physicssim.demo;

import com.physicssim.Force;
import com.physicssim.PhysicsSimulator;
import com.physicssim.RigidBody;
import com.physicssim.Shapes;
import com.physicssim.Vector2D;

import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Spawns a batch of random rigid bodies (random shape, random scale) above the floor,
 * lets them fall and pile up, then resets after SCENE_DURATION seconds and repeats.
 */
public class TestScene extends JPanel {
    private final List<RigidBody> rigidBodies = new ArrayList<>();
    private final TestingRenderer renderer = new TestingRenderer();
    private final PhysicsSimulator physicsSimulator = new PhysicsSimulator();
    private final Random rng = new Random();

    private static final int TARGET_FPS = 60;
    private static final double FRAME_TIME = 1.0 / TARGET_FPS;

    private static final int PHYSICS_STEPS_PER_SECOND = 240;
    private static final double PHYSICS_STEP_TIME = 1.0 / PHYSICS_STEPS_PER_SECOND;

    // Scale so the full arena width (walls at ±8.3, plus a margin) fits on screen.
    // 20 world-units across means ±10 visible, giving ~1.5-unit clearance outside each wall.
    private static final double PIXELS_PER_UNIT =
            Toolkit.getDefaultToolkit().getScreenSize().width / 20.0;

    private static final double GRAVITY = 9.8;
    private static final int DYNAMIC_BODY_COUNT = 20;
    private static final double SCENE_DURATION = 6.0;

    // Number of static boundary bodies kept across resets
    private int staticBodyCount = 0;
    private double sceneTimer = 0.0;

    public TestScene() {
        setupScene();
        setBackground(Color.WHITE);
    }

    private void setupScene() {
        // Immovable floor spanning the visible width
        Vector2D[] floorVerts = {
            new Vector2D(-9, -0.3), new Vector2D(9, -0.3),
            new Vector2D( 9,  0.3), new Vector2D(-9, 0.3)
        };
        rigidBodies.add(RigidBody.createStatic(floorVerts, new Vector2D(0, -5), 0));

        // Side walls
        Vector2D[] wallVerts = {
            new Vector2D(-0.3, -7), new Vector2D(0.3, -7),
            new Vector2D( 0.3,  7), new Vector2D(-0.3, 7)
        };
        rigidBodies.add(RigidBody.createStatic(wallVerts, new Vector2D(-8.3, 0), 0));
        rigidBodies.add(RigidBody.createStatic(wallVerts, new Vector2D( 8.3, 0), 0));

        staticBodyCount = rigidBodies.size();
        spawnObjects();
    }

    private void spawnObjects() {
        for (int i = 0; i < DYNAMIC_BODY_COUNT; i++) {
            spawnRandomBody();
        }
    }

    /** Called only from the physics thread while it already holds the rigidBodies lock. */
    private void resetSceneLocked() {
        while (rigidBodies.size() > staticBodyCount) {
            rigidBodies.remove(rigidBodies.size() - 1);
        }
        sceneTimer = 0.0;
        spawnObjects();
    }

    private void spawnRandomBody() {
        double x = -6.5 + rng.nextDouble() * 13.0;
        double y = 1.5 + rng.nextDouble() * 6.0;
        double orientation = rng.nextDouble() * Math.PI * 2;
        double scale = 0.35 + rng.nextDouble() * 0.65;

        Vector2D[] vertices = randomShape(scale);
        double mass = 0.5 + rng.nextDouble() * 2.5;

        RigidBody body = new RigidBody(vertices, mass, new Vector2D(x, y), orientation);
        body.setVelocity(new Vector2D((rng.nextDouble() - 0.5) * 3.0, 0));
        body.setAngularVelocity((rng.nextDouble() - 0.5) * 4.0);
        addGravity(body, mass);
        rigidBodies.add(body);
    }

    private Vector2D[] randomShape(double scale) {
        switch (rng.nextInt(8)) {
            case 0:  // rectangle with random aspect ratio
                return Shapes.rectangle(
                    scale * (0.5 + rng.nextDouble() * 0.8),
                    scale * (0.3 + rng.nextDouble() * 0.5));
            case 1:  // regular polygon: triangle through octagon
                return Shapes.regularPolygon(3 + rng.nextInt(6), scale);
            case 2:  // star
                return Shapes.star(4 + rng.nextInt(3), scale, scale * (0.28 + rng.nextDouble() * 0.22));
            case 3:  // isoceles triangle (thinner/taller than regularPolygon(3))
                return Shapes.triangle(
                    scale * (0.4 + rng.nextDouble() * 0.5),
                    scale * (1.0 + rng.nextDouble() * 0.8));
            case 4:  // pill / capsule (octagonal, rolls smoothly)
                return Shapes.capsule(
                    scale * (0.9 + rng.nextDouble() * 0.6),
                    scale * (0.3 + rng.nextDouble() * 0.2));
            case 5:  // elongated diamond (pointy, slides and spins nicely)
                return Shapes.diamond(
                    scale * (0.3 + rng.nextDouble() * 0.3),
                    scale * (0.8 + rng.nextDouble() * 0.6));
            case 6:  // parallelogram (slanted rectangle, interesting friction)
                return Shapes.parallelogram(
                    scale * (0.6 + rng.nextDouble() * 0.5),
                    scale * (0.25 + rng.nextDouble() * 0.3),
                    scale * (0.3 + rng.nextDouble() * 0.3));
            default: // cross / plus sign (concave, tumbles dramatically)
                return Shapes.cross(
                    scale * (0.7 + rng.nextDouble() * 0.3),
                    scale * (0.25 + rng.nextDouble() * 0.15));
        }
    }

    private void addGravity(RigidBody body, double mass) {
        body.applyForce(new Force(new Vector2D(0, -mass * GRAVITY), new Vector2D(0, 0)));
    }

    // ── Rendering ────────────────────────────────────────────────────────────

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        synchronized (rigidBodies) {
            renderer.drawRigidBodies(g2d, rigidBodies, getWidth() / 2.0, getHeight() / 2.0, PIXELS_PER_UNIT);
        }
    }

    // ── Loop ─────────────────────────────────────────────────────────────────

    /** Render loop: repaints at TARGET_FPS, no physics. */
    public void startRenderLoop() {
        Thread renderThread = new Thread(() -> {
            long lastFrameTime = System.nanoTime();
            long frameTimeNanos = (long) (FRAME_TIME * 1_000_000_000L);

            while (true) {
                long currentTime = System.nanoTime();
                if (currentTime - lastFrameTime >= frameTimeNanos) {
                    repaint();
                    lastFrameTime = currentTime;
                }
                try {
                    long sleepTime = frameTimeNanos - (System.nanoTime() - lastFrameTime);
                    if (sleepTime > 0) Thread.sleep(sleepTime / 1_000_000);
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
     * Physics thread: steps the simulation at exactly PHYSICS_STEPS_PER_SECOND Hz,
     * independent of the render rate. Uses a fixed-advance clock so steps never drift.
     */
    public void startPhysicsThread() {
        Thread physicsThread = new Thread(() -> {
            long last = System.nanoTime();
            long stepNs = (long) (PHYSICS_STEP_TIME * 1_000_000_000L);

            while (true) {
                long now = System.nanoTime();
                if (now - last >= stepNs) {
                    synchronized (rigidBodies) {
                        sceneTimer += PHYSICS_STEP_TIME;
                        if (sceneTimer >= SCENE_DURATION) resetSceneLocked();
                        physicsSimulator.simulateTimeStep(rigidBodies, PHYSICS_STEP_TIME);
                    }
                    last += stepNs;
                    // Guard against spiral-of-death if the thread is stalled for a long time
                    if (System.nanoTime() - last > 2 * stepNs) last = System.nanoTime();
                }
                try {
                    Thread.sleep(1);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        });
        physicsThread.setDaemon(true);
        physicsThread.start();
    }

    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> {
            JFrame frame = new JFrame("Physics Simulation");
            TestScene scene = new TestScene();
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            frame.setUndecorated(true);
            frame.add(scene);
            frame.setExtendedState(JFrame.MAXIMIZED_BOTH);
            frame.setVisible(true);
            // ESC to exit
            scene.getInputMap(WHEN_IN_FOCUSED_WINDOW)
                 .put(javax.swing.KeyStroke.getKeyStroke(java.awt.event.KeyEvent.VK_ESCAPE, 0), "exit");
            scene.getActionMap().put("exit", new javax.swing.AbstractAction() {
                public void actionPerformed(java.awt.event.ActionEvent e) { System.exit(0); }
            });
            scene.startPhysicsThread();
            scene.startRenderLoop();
        });
    }
}
