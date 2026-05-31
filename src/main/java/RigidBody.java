import java.util.ArrayList;
import java.util.List;

public class RigidBody {
    private final Vector2D[] vertices;

    private final double mass;
    private final double momentOfInertia; // About the body-space origin
    private final double coefficientOfRestitution; // Bounciness (0 = perfectly inelastic, 1 = perfectly elastic)

    private Vector2D velocity; // In world-space
    private Vector2D worldPos;
    private double angularVelocity; // In body-space, around the origin (0,0)
    private double worldOrientation; // in radians

    private Vector2D linearAcceleration; // In world-space, computed from applied forces
    private double angularAcceleration; // In body-space, computed from applied forces

    private final ArrayList<Force> appliedForces = new ArrayList<>();

    public RigidBody(Vector2D[] vertices, double mass, Vector2D pos, double orientation) {
        this(vertices, mass, pos, orientation, 0.8); // Default restitution of 0.8
    }

    public RigidBody(Vector2D[] vertices, double mass, Vector2D pos, double orientation, double coefficientOfRestitution) {
        this.vertices = vertices;
        this.mass = mass;
        this.coefficientOfRestitution = coefficientOfRestitution;

        // Compute inertia about the body-space origin (do NOT translate vertices by COM)
        this.momentOfInertia = calculateMomentOfInertia(vertices, mass);

        this.velocity = new Vector2D(0, 0);
        this.worldPos = pos;
        this.angularVelocity = 0;
        this.worldOrientation = orientation;

        this.linearAcceleration = new Vector2D(0, 0);
        this.angularAcceleration = 0;
    }

    /**
     * Calculate the world position of the body after a given time interval, assuming constant acceleration based on the currently applied forces.
     */
    private Vector2D calculateWorldPosAfterTime(double time) {
        // s = ut + 0.5at^2
        Vector2D displacement = velocity.scale(time).add(linearAcceleration.scale(0.5 * time * time));
        return worldPos.add(displacement);
    }

    /**
     * Calculate the world orientation of the body after a given time interval, assuming constant angular acceleration based on the currently applied forces.
     */
    private double calculateWorldOrientationAfterTime(double time) {
        // θ = ωt + 0.5αt^2
        return worldOrientation + angularVelocity * time + 0.5 * angularAcceleration * time * time;
    }

    public void stepTime (double time) {
        // Update accelerations based on currently applied forces
        this.linearAcceleration = getLinearAcceleration();
        this.angularAcceleration = getAngularAcceleration();

        // Update position and orientation based on current velocity and acceleration
        this.worldPos = calculateWorldPosAfterTime(time);
        this.worldOrientation = calculateWorldOrientationAfterTime(time);

        // Update velocities based on acceleration
        this.velocity = velocity.add(linearAcceleration.scale(time));
        this.angularVelocity += angularAcceleration * time;
    }

    public void applyForce(Force f) {
        this.appliedForces.add(f);
    }

    public void clearForces() {
        this.appliedForces.clear();
    }

    public Vector2D getLinearAcceleration() {
        return calculateLinearAcceleration();
    }

    public double getAngularAcceleration() {
        return calculateAngularAcceleration();
    }

    public Vector2D getVelocity() {
        return velocity;
    }

    public void setVelocity(Vector2D velocity) {
        this.velocity = velocity;
    }

    public double getAngularVelocity() {
        return angularVelocity;
    }

    public void setAngularVelocity(double angularVelocity) {
        this.angularVelocity = angularVelocity;
    }

    public double getMass () {
        return mass;
    }

    public double getMomentOfInertia() {
        return momentOfInertia;
    }

    public double getCoefficientOfRestitution() {
        return coefficientOfRestitution;
    }

    public Vector2D[] getVertices() {
        return vertices;
    }

    public List<Force> getAppliedForces() {
        return appliedForces;
    }

    public Vector2D getWorldPos() {
        return worldPos;
    }

    public void setWorldPos(Vector2D worldPos) {
        this.worldPos = worldPos;
    }

    public double getWorldOrientation() {
        return worldOrientation;
    }

    /**
     * Calculate the angular acceleration based on the applied forces and their application points.
     * Rotations and torques are computed about the body-space origin: application points are treated
     * as positions relative to the body origin, rotated into world-space, and then crossed with the
     * world-space force to compute torque about that origin.
     */
    private double calculateAngularAcceleration(){
        if (Math.abs(momentOfInertia) < 1e-12) {
            throw new IllegalStateException("Moment of inertia is zero or too small to compute angular acceleration");
        }

        double netTorque = 0.0;

        for (Force force : appliedForces) {
            Vector2D rBody = force.getApplicationPoint(); // body-space (relative to body origin)
            Vector2D fWorld = force.getForceVector();     // world-space

            // Rotate r from body-space origin into world-space: r_world = R(orientation) * r_body
            Vector2D rWorld = rBody.rotate(worldOrientation);

            // Torque = r_world x f_world (scalar z-component) — torque about the body origin
            double torque = rWorld.cross(fWorld);
            netTorque += torque;
        }

        return netTorque / momentOfInertia; // α = τ / I_origin
    }

    /**
     * Calculate the linear acceleration based on the net force applied to the body.
     * This is in world-space: sum all world-space forces and divide by mass.
     */
    private Vector2D calculateLinearAcceleration(){
        if (Math.abs(mass) < 1e-12) {
            throw new IllegalStateException("Mass is zero or too small to compute linear acceleration");
        }

        Vector2D netForce = new Vector2D(0, 0);
        for (Force force : appliedForces) {
            netForce = netForce.add(force.getForceVector());
        }
        return netForce.scale(1.0 / mass); // a = F / m
    }

    /**
     * Calculate the moment of inertia for a polygon defined by its vertices and mass.
     * This assumes the object has uniform density and is a simple polygon (not self-intersecting).
     * The moment of inertia is calculated about the origin (0,0) in body-space.
     */
    private static double calculateMomentOfInertia(Vector2D[] vertices, double mass) {
        double twiceArea = 0;
        double inertiaSum = 0;

        for (int i = 0; i < vertices.length; i++) {
            Vector2D v0 = vertices[i];
            Vector2D v1 = vertices[(i + 1) % vertices.length]; // Wrap around to the first vertex

            double cross = v0.cross(v1);
            twiceArea += cross;
            inertiaSum += (v0.getX() * v0.getX() + v0.getX() * v1.getX() + v1.getX() * v1.getX()
                    + v0.getY() * v0.getY() + v0.getY() * v1.getY() + v1.getY() * v1.getY()) * cross;
        }

        double area = 0.5 * twiceArea;
        if (Math.abs(area) < 1e-12) {
            throw new IllegalArgumentException("Polygon area is zero or degenerate");
        }

        // Area moment formula: I_area = (1/12) * sum(cross * term)
        // Convert to mass moment: I_mass = (mass / area) * I_area = mass/(12*area) * inertiaSum
        return (mass / (12.0 * area)) * inertiaSum;
    }
}
