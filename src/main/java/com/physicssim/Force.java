package com.physicssim;

public class Force {
    private final Vector2D forceVector; // In world-space
    private final Vector2D applicationPoint; // Relative to the origin in body-space

    public Force(Vector2D forceVector, Vector2D applicationPoint) {
        this.forceVector = forceVector;
        this.applicationPoint = applicationPoint;
    }

    public Vector2D getForceVector() {
        return forceVector;
    }

    public Vector2D getApplicationPoint() {
        return applicationPoint;
    }
}
