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

    public static Vector2D[] regularPolygon(int sides, double radius) {
        Vector2D[] verts = new Vector2D[sides];
        for (int i = 0; i < sides; i++) {
            double angle = 2 * Math.PI * i / sides;
            verts[i] = new Vector2D(radius * Math.cos(angle), radius * Math.sin(angle));
        }
        return verts;
    }

    public static Vector2D[] triangle(double halfWidth, double height) {
        return new Vector2D[]{
            new Vector2D(-halfWidth, 0),
            new Vector2D( halfWidth, 0),
            new Vector2D( 0, height)
        };
    }

    public static Vector2D[] star(int points, double innerRadius, double outerRadius) {
        Vector2D[] verts = new Vector2D[points * 2];
        for (int i = 0; i < points * 2; i++) {
            double angle = Math.PI * i / points;
            double radius = (i % 2 == 0) ? outerRadius : innerRadius;
            verts[i] = new Vector2D(radius * Math.cos(angle), radius * Math.sin(angle));
        }
        return verts;
    }
}
