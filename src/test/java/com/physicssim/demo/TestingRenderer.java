package com.physicssim.demo;

import com.physicssim.Force;
import com.physicssim.RigidBody;
import com.physicssim.Vector2D;

import java.awt.*;
import java.util.List;

/**
 * This class is a placeholder for testing physics simulation.
 * It draws rigid bodies using simple shapes and colors.
 *
 * Transforms rigid bodies from world-space to screen-space using position, scale, and rotation.
 */
public class TestingRenderer {
    private static final int VERTEX_RADIUS = 3; // Radius of the red circles at vertices

    // Force visualization settings (all in pixels unless otherwise noted)
    private static final int FORCE_POINT_RADIUS = 4;            // small circle at application point
    private static final double FORCE_VISUAL_SCALE = 0.05;      // multiplier: force (N) -> fraction of pixelsPerUnit
    private static final int ARROW_HEAD_SIZE = 8;               // pixels

    /**
     * Draws a list of rigid bodies onto a Graphics2D context.
     *
     * Each rigid body is rendered with:
     * - Red circles at each vertex
     * - Blue line segments connecting vertices in order
     * - For each applied force: a small circle at the application point and a magenta arrow for the force vector
     *
     * @param graphics The Graphics2D context to draw on
     * @param rigidBodies The list of rigid bodies to render
     * @param screenCenterX The x-coordinate of the screen center (world origin in screen space)
     * @param screenCenterY The y-coordinate of the screen center (world origin in screen space)
     * @param pixelsPerUnit The scale factor (pixels per world unit)
     */
    public void drawRigidBodies(Graphics2D graphics, List<RigidBody> rigidBodies,
                                 double screenCenterX, double screenCenterY, double pixelsPerUnit) {
        for (RigidBody body : rigidBodies) {
            drawRigidBody(graphics, body, screenCenterX, screenCenterY, pixelsPerUnit);
        }
    }

    /**
     * Draws a single rigid body onto a Graphics2D context.
     *
     * @param graphics The Graphics2D context to draw on
     * @param body The rigid body to render
     * @param screenCenterX The x-coordinate of the screen center (world origin in screen space)
     * @param screenCenterY The y-coordinate of the screen center (world origin in screen space)
     * @param pixelsPerUnit The scale factor (pixels per world unit)
     */
    private void drawRigidBody(Graphics2D graphics, RigidBody body,
                                double screenCenterX, double screenCenterY, double pixelsPerUnit) {
        Vector2D[] vertices = body.getVertices();

        if (vertices.length == 0) {
            return;
        }

        // Transform vertices from body-space to world-space to screen-space
        Vector2D[] screenVertices = new Vector2D[vertices.length];
        for (int i = 0; i < vertices.length; i++) {
            screenVertices[i] = transformToScreenSpace(vertices[i], body.getWorldPos(),
                                                        body.getWorldOrientation(),
                                                        screenCenterX, screenCenterY, pixelsPerUnit);
        }

        // Draw blue line segments connecting vertices in order
        graphics.setColor(Color.BLUE);
        graphics.setStroke(new BasicStroke(2.0f));
        for (int i = 0; i < screenVertices.length; i++) {
            int nextI = (i + 1) % screenVertices.length;
            drawLine(graphics, screenVertices[i], screenVertices[nextI]);
        }

        // Draw red circles at each vertex
        graphics.setColor(Color.RED);
        for (Vector2D screenVertex : screenVertices) {
            drawCircle(graphics, screenVertex, VERTEX_RADIUS);
        }

        // Draw applied forces: a small circle at the application point and an arrow representing the force vector
        List<Force> forces = body.getAppliedForces();
        if (forces != null && !forces.isEmpty()) {
            for (Force f : forces) {
                // Application point is in body-space; transform to screen-space
                Vector2D appPointBody = f.getApplicationPoint();
                Vector2D appPointScreen = transformToScreenSpace(appPointBody, body.getWorldPos(),
                                                                 body.getWorldOrientation(),
                                                                 screenCenterX, screenCenterY, pixelsPerUnit);

                // Draw application point
                graphics.setColor(Color.GREEN.darker());
                drawCircle(graphics, appPointScreen, FORCE_POINT_RADIUS);

                // Force vector is given in world-space. Convert to screen-space displacement.
                Vector2D fWorld = f.getForceVector();
                double dx = fWorld.getX() * pixelsPerUnit * FORCE_VISUAL_SCALE;
                double dy = -fWorld.getY() * pixelsPerUnit * FORCE_VISUAL_SCALE; // invert Y for screen

                Vector2D forceEnd = new Vector2D(appPointScreen.getX() + dx, appPointScreen.getY() + dy);

                // Draw arrow line
                graphics.setColor(Color.MAGENTA);
                graphics.setStroke(new BasicStroke(2.0f));
                drawLine(graphics, appPointScreen, forceEnd);

                // Draw arrowhead
                drawArrowHead(graphics, appPointScreen, forceEnd, ARROW_HEAD_SIZE);
            }
        }
    }

    /**
     * Draws an arrow head at the end of a vector from start -> end
     */
    private void drawArrowHead(Graphics2D g, Vector2D start, Vector2D end, int size) {
        double angle = Math.atan2(end.getY() - start.getY(), end.getX() - start.getX());
        double angle1 = angle + Math.toRadians(20);
        double angle2 = angle - Math.toRadians(20);

        int x = (int) Math.round(end.getX());
        int y = (int) Math.round(end.getY());

        int x1 = (int) Math.round(x - size * Math.cos(angle1));
        int y1 = (int) Math.round(y - size * Math.sin(angle1));

        int x2 = (int) Math.round(x - size * Math.cos(angle2));
        int y2 = (int) Math.round(y - size * Math.sin(angle2));

        g.drawLine(x, y, x1, y1);
        g.drawLine(x, y, x2, y2);
    }

    /**
     * Transforms a point from body-space to screen-space.
     *
     * Applies rotation (around the rigid body's world position) and then translation to world-space,
     * followed by scaling and translation to screen-space.
     *
     * @param bodySpaceVertex The vertex in body-space
     * @param worldPos The rigid body's position in world-space
     * @param worldOrientation The rigid body's orientation in radians
     * @param screenCenterX The x-coordinate of the screen center
     * @param screenCenterY The y-coordinate of the screen center
     * @param pixelsPerUnit The scale factor
     * @return The vertex in screen-space
     */
    private Vector2D transformToScreenSpace(Vector2D bodySpaceVertex, Vector2D worldPos,
                                             double worldOrientation, double screenCenterX,
                                             double screenCenterY, double pixelsPerUnit) {
        // Rotate vertex around body origin
        Vector2D rotatedVertex = bodySpaceVertex.rotate(worldOrientation);

        // Translate to world-space
        Vector2D worldSpaceVertex = rotatedVertex.add(worldPos);

        // Scale to screen-space (with Y-axis inverted for screen coordinates)
        double screenX = screenCenterX + worldSpaceVertex.getX() * pixelsPerUnit;
        double screenY = screenCenterY - worldSpaceVertex.getY() * pixelsPerUnit; // Invert Y-axis

        return new Vector2D(screenX, screenY);
    }

    /**
     * Draws a line segment between two screen-space points.
     *
     * @param graphics The Graphics2D context
     * @param start The starting point
     * @param end The ending point
     */
    private void drawLine(Graphics2D graphics, Vector2D start, Vector2D end) {
        int x1 = (int) Math.round(start.getX());
        int y1 = (int) Math.round(start.getY());
        int x2 = (int) Math.round(end.getX());
        int y2 = (int) Math.round(end.getY());
        graphics.drawLine(x1, y1, x2, y2);
    }

    /**
     * Draws a filled circle at the given screen-space position.
     *
     * @param graphics The Graphics2D context
     * @param center The center point of the circle
     * @param radius The radius of the circle
     */
    private void drawCircle(Graphics2D graphics, Vector2D center, int radius) {
        int x = (int) Math.round(center.getX());
        int y = (int) Math.round(center.getY());
        graphics.fillOval(x - radius, y - radius, radius * 2, radius * 2);
    }
}
