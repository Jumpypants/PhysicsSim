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
        // Create a square rigid body
        Vector2D[] squareVertices = new Vector2D[]{
            new Vector2D(-0.5, -0.5),  // Bottom-left
            new Vector2D(0.5, -0.5),   // Bottom-right
            new Vector2D(0.5, 0.5),    // Top-right
            new Vector2D(-0.5, 0.5)    // Top-left
        };

        // Create the first square at origin with mass 1.0 and no rotation
        Vector2D initialPos1 = new Vector2D(0, -1);
        double mass = 1.0;
        double initialOrientation = 0.0;

        RigidBody square1 = new RigidBody(squareVertices, mass, initialPos1, initialOrientation);

        // Apply downward gravitational force at the center of mass (origin in body-space)
        // Force = mass * g, directed downward (negative Y direction)
        Vector2D forceVector = new Vector2D(0, -mass * GRAVITY);
        Vector2D applicationPoint = new Vector2D(0, 0); // Center of mass in body-space
        Force gravityForce = new Force(forceVector, applicationPoint);
        square1.applyForce(gravityForce);

        rigidBodies.add(square1);

        // Create a second square body positioned directly above square1, with downward velocity to ensure collision
        Vector2D initialPos2 = new Vector2D(0, 2);
        RigidBody square2 = new RigidBody(squareVertices, mass, initialPos2, initialOrientation);

        // Set initial downward velocity to ensure collision
        square2.setVelocity(new Vector2D(0, -3.0)); // Moving downward at 3 m/s

        // Apply gravity to the second square as well
        Force gravityForce2 = new Force(forceVector, applicationPoint);
        square2.applyForce(gravityForce2);

        rigidBodies.add(square2);
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
