package com.physicssim;

public class Shapes {
    private Shapes() {}

    public static Vector2D[] rectangle(double halfWidth, double halfHeight) {
        return new Vector2D[]{
            new Vector2D(-halfWidth, -halfHeight),
            new Vector2D( halfWidth, -halfHeight),
            new Vector2D( halfWidth,  halfHeight),
            new Vector2D(-halfWidth,  halfHeight)
        };
    }

    /** Isoceles triangle: base centred on the x-axis, apex pointing up. */
    public static Vector2D[] triangle(double halfWidth, double height) {
        return new Vector2D[]{
            new Vector2D(-halfWidth, 0),
            new Vector2D( halfWidth, 0),
            new Vector2D(0, height)
        };
    }

    /** Regular n-gon wound CCW. */
    public static Vector2D[] regularPolygon(int sides, double radius) {
        Vector2D[] verts = new Vector2D[sides];
        for (int i = 0; i < sides; i++) {
            double angle = 2 * Math.PI * i / sides;
            verts[i] = new Vector2D(radius * Math.cos(angle), radius * Math.sin(angle));
        }
        return verts;
    }

    /** Star polygon wound CCW, first tip pointing up. */
    public static Vector2D[] star(int points, double outerRadius, double innerRadius) {
        int n = points * 2;
        Vector2D[] verts = new Vector2D[n];
        for (int i = 0; i < n; i++) {
            double angle = Math.PI / 2.0 + 2.0 * Math.PI * i / n;
            double r = (i % 2 == 0) ? outerRadius : innerRadius;
            verts[i] = new Vector2D(r * Math.cos(angle), r * Math.sin(angle));
        }
        return verts;
    }

    /** Octagonal pill: a rectangle with all four corners chamfered. */
    public static Vector2D[] capsule(double halfLength, double halfWidth) {
        double cut = halfWidth * 0.6;
        return new Vector2D[]{
            new Vector2D(-halfLength + cut, -halfWidth),
            new Vector2D( halfLength - cut, -halfWidth),
            new Vector2D( halfLength,       -halfWidth + cut),
            new Vector2D( halfLength,        halfWidth - cut),
            new Vector2D( halfLength - cut,  halfWidth),
            new Vector2D(-halfLength + cut,  halfWidth),
            new Vector2D(-halfLength,        halfWidth - cut),
            new Vector2D(-halfLength,       -halfWidth + cut)
        };
    }

    /** Four-sided diamond with independent half-extents on each axis. */
    public static Vector2D[] diamond(double halfWidth, double halfHeight) {
        return new Vector2D[]{
            new Vector2D( 0,          -halfHeight),
            new Vector2D( halfWidth,   0),
            new Vector2D( 0,           halfHeight),
            new Vector2D(-halfWidth,   0)
        };
    }

    /** Parallelogram: rectangle sheared horizontally by {@code shear} units. */
    public static Vector2D[] parallelogram(double halfWidth, double halfHeight, double shear) {
        return new Vector2D[]{
            new Vector2D(-halfWidth + shear, -halfHeight),
            new Vector2D( halfWidth + shear, -halfHeight),
            new Vector2D( halfWidth - shear,  halfHeight),
            new Vector2D(-halfWidth - shear,  halfHeight)
        };
    }

    /** Plus / cross shape (concave, 12 vertices). */
    public static Vector2D[] cross(double armLength, double halfArmWidth) {
        double h = halfArmWidth;
        double l = armLength;
        return new Vector2D[]{
            new Vector2D(-h, -l),
            new Vector2D( h, -l),
            new Vector2D( h, -h),
            new Vector2D( l, -h),
            new Vector2D( l,  h),
            new Vector2D( h,  h),
            new Vector2D( h,  l),
            new Vector2D(-h,  l),
            new Vector2D(-h,  h),
            new Vector2D(-l,  h),
            new Vector2D(-l, -h),
            new Vector2D(-h, -h)
        };
    }
}
