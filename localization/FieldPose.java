package org.firstinspires.ftc.teamcode.localization;

/**
 * Immutable 2D pose expressed in the field frame.
 *
 * Units: inches and radians.
 * Heading is CCW positive and measured from the +X axis, the same convention
 * used by every other class in this localization package.
 *
 * The field frame is defined in {@link FieldConstants}:
 * origin at a field corner, +X along one edge, +Y along the other edge,
 * so the whole 144 x 144 inch field is [0, 144] x [0, 144].
 */
public class FieldPose {

    public static final FieldPose ZERO = new FieldPose(0.0, 0.0, 0.0);

    public final double x;
    public final double y;
    public final double heading;

    public FieldPose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = normalizeAngle(heading);
    }

    /** Convenience constructor that takes the heading in degrees. */
    public static FieldPose ofDegrees(double x, double y, double headingDeg) {
        return new FieldPose(x, y, Math.toRadians(headingDeg));
    }

    public FieldPose withHeading(double newHeading) {
        return new FieldPose(x, y, newHeading);
    }

    public FieldPose plus(double dx, double dy, double dHeading) {
        return new FieldPose(x + dx, y + dy, heading + dHeading);
    }

    public double distanceTo(FieldPose other) {
        double dx = other.x - x;
        double dy = other.y - y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    /** Field-frame bearing from this pose toward the given point. */
    public double bearingTo(FieldPose other) {
        return Math.atan2(other.y - y, other.x - x);
    }

    /** Signed shortest angular error from this heading to the target heading. */
    public double headingError(double targetHeading) {
        return normalizeAngle(targetHeading - heading);
    }

    /**
     * Mirrors the pose across the field centerline, which is what an alliance
     * flip looks like for a 180-deg symmetric field.
     */
    public FieldPose mirrorAcrossField() {
        return new FieldPose(
                FieldConstants.FIELD_SIZE_IN - x,
                y,
                Math.PI - heading);
    }

    /** Wraps an angle into (-PI, PI]. */
    public static double normalizeAngle(double angle) {
        double wrapped = angle % (2.0 * Math.PI);
        if (wrapped > Math.PI) {
            wrapped -= 2.0 * Math.PI;
        } else if (wrapped <= -Math.PI) {
            wrapped += 2.0 * Math.PI;
        }
        return wrapped;
    }

    public static FieldPose interpolate(FieldPose a, FieldPose b, double t) {
        double deltaHeading = normalizeAngle(b.heading - a.heading);
        return new FieldPose(
                a.x + (b.x - a.x) * t,
                a.y + (b.y - a.y) * t,
                a.heading + deltaHeading * t);
    }

    @Override
    public String toString() {
        return String.format("(%.2f, %.2f, %.1fdeg)", x, y, Math.toDegrees(heading));
    }
}
