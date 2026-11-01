package org.firstinspires.ftc.teamcode.localization;

/**
 * Field-frame definition and alliance handling.
 *
 * FRAME CONVENTION (shared by odometry, vision and path following):
 *   - Origin (0, 0) is one corner of the field floor.
 *   - +X runs along one field edge, +Y along the other edge.
 *   - Heading 0 deg points along +X, positive rotation is CCW (math positive).
 *   - Units are inches everywhere. No meters are used above the Limelight layer.
 *
 * If your robot's "forward" at heading 0 does not match your mental picture of
 * the field, you only have to change the START pose and the waypoint poses in
 * the auto OpMode; nothing else in the localization stack needs to be touched.
 */
public class FieldConstants {

    /** FTC field is 12 ft x 12 ft = 144 in x 144 in. */
    public static final double FIELD_SIZE_IN = 144.0;

    /** Half of the field, handy for mirroring and for sanity checks. */
    public static final double FIELD_CENTER_IN = FIELD_SIZE_IN / 2.0;

    /**
     * Poses outside of this margin are almost certainly a localization error,
     * so the telemetry layer flags them and the auto layer can refuse to drive.
     */
    public static final double OUT_OF_BOUNDS_MARGIN_IN = 18.0;

    private FieldConstants() {
    }

    /** Returns true when the pose is physically plausible for a robot on this field. */
    public static boolean isInsideField(FieldPose pose) {
        return pose.x > -OUT_OF_BOUNDS_MARGIN_IN
                && pose.x < FIELD_SIZE_IN + OUT_OF_BOUNDS_MARGIN_IN
                && pose.y > -OUT_OF_BOUNDS_MARGIN_IN
                && pose.y < FIELD_SIZE_IN + OUT_OF_BOUNDS_MARGIN_IN;
    }

    /** Keeps a pose inside the legal field area, used against vision glitches. */
    public static FieldPose clampToField(FieldPose pose) {
        double clampedX = Math.max(0.0, Math.min(FIELD_SIZE_IN, pose.x));
        double clampedY = Math.max(0.0, Math.min(FIELD_SIZE_IN, pose.y));
        return new FieldPose(clampedX, clampedY, pose.heading);
    }

    /** Red and blue alliance home sides, used by the auto OpMode defaults. */
    public enum Alliance {
        RED,
        BLUE
    }
}
