package org.firstinspires.ftc.teamcode.localization;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * AprilTag field map: tag ID -> where that tag lives in the field frame.
 *
 * WHY THIS EXISTS
 * The Limelight can already output a robot pose (MegaTag / MegaTag2), but that
 * pose is expressed in whatever field map is uploaded to the Limelight itself.
 * Owning the map here means the robot pose is computed against positions you
 * control, in the exact inches/CCW frame used by odometry and path following,
 * and it keeps working even when nobody uploaded a field map to the camera.
 *
 * ---------------------------------------------------------------------------
 * BIOBUZZ 2026-2027 FIELD (from the official manual, section 9)
 * ---------------------------------------------------------------------------
 *   - Tag family: 36h11, tag IDs 30 ... 45 (16 tags, four clusters of four).
 *   - Cluster 30-33: audience side, RED hive.
 *   - Cluster 34-37: audience side, BLUE hive.
 *   - Cluster 38-41: far side (opposite the audience), RED hive.
 *   - Cluster 42-45: far side, BLUE hive.
 *   - Every tag sits on the BOTTOM of a hive, facing DOWN at the mat, with its
 *     bottom edge pointing at the field center.
 *
 * IMPORTANT CONSEQUENCE OF "FACING DOWN"
 * Because the tags look down, a camera on the robot has to look UP at them, and
 * the tag face is not perpendicular to the camera's optical axis. The tag's
 * horizontal position is still recovered correctly (the solver only uses the
 * measured x/y offset), but the tag's ROLL/YAW as seen by the camera is not the
 * robot's yaw any more. That is why {@code VisionConstants.USE_TAG_HEADING} is
 * false for this game: heading is owned by the IMU and the tags only fix
 * position. Do not turn it back on without re-deriving the relation.
 *
 * WHAT YOU STILL HAVE TO FILL IN
 * The manual pages describe the geometry INSIDE a cluster (6.5 in / 7.0 in
 * spacing, 2.75 in margin, the 3.25 in reference hole) but not where the four
 * hives sit on the 144 x 144 in field. Measure the four anchor points below
 * with a tape measure, or read them off the field drawing, and the 16 tags are
 * generated automatically.
 */
public class AprilTagMap {

    public static class Tag {
        public final int id;
        public final double x;
        public final double y;
        public final double yaw;

        public Tag(int id, double x, double y, double yawRad) {
            this.id = id;
            this.x = x;
            this.y = y;
            this.yaw = yawRad;
        }

        public FieldPose toPose() {
            return new FieldPose(x, y, yaw);
        }

        @Override
        public String toString() {
            return String.format("Tag %d @ (%.1f, %.1f) yaw %.0f", id, x, y, Math.toDegrees(yaw));
        }
    }

    // ------------------------------------------------------------------
    // CLUSTER GEOMETRY (from the manual, per hive)
    // ------------------------------------------------------------------

    /** Center-to-center spacing between tag 1-2 and 3-4 of a cluster, inches. */
    public static double CLUSTER_SPACING_OUTER_IN = 6.5;

    /** Center-to-center spacing across the middle pair of a cluster, inches. */
    public static double CLUSTER_SPACING_MIDDLE_IN = 7.0;

    // ------------------------------------------------------------------
    // HIVE ANCHORS - EDIT THESE FOUR PAIRS
    // x/y is the CENTER of that hive's four-tag row, in inches, field frame.
    // The yaw is the direction the tag row's bottom edge points, i.e. toward
    // the field center, exactly as the manual specifies.
    // The values below are ESTIMATES for a symmetric layout: verify with a
    // tape measure before trusting auto. Wrong numbers here give a pose that
    // is consistently wrong, never randomly noisy.
    // ------------------------------------------------------------------

    /** Audience side is treated as the y = 0 edge. */
    public static double AUDIENCE_RED_X = 36.0;
    public static double AUDIENCE_RED_Y = 12.0;
    public static double AUDIENCE_RED_YAW_DEG = 90.0;

    public static double AUDIENCE_BLUE_X = 108.0;
    public static double AUDIENCE_BLUE_Y = 12.0;
    public static double AUDIENCE_BLUE_YAW_DEG = 90.0;

    /** Far side is the y = 144 edge. */
    public static double FAR_RED_X = 36.0;
    public static double FAR_RED_Y = 132.0;
    public static double FAR_RED_YAW_DEG = -90.0;

    public static double FAR_BLUE_X = 108.0;
    public static double FAR_BLUE_Y = 132.0;
    public static double FAR_BLUE_YAW_DEG = -90.0;

    private static final Map<Integer, Tag> TAGS = new LinkedHashMap<>();

    static {
        rebuild();
    }

    /**
     * Regenerates all 16 tags from the four anchors. Call it after editing the
     * anchor values at runtime (a tuning OpMode can do that live).
     */
    public static void rebuild() {
        TAGS.clear();
        addCluster(new int[]{30, 31, 32, 33}, AUDIENCE_RED_X, AUDIENCE_RED_Y, AUDIENCE_RED_YAW_DEG);
        addCluster(new int[]{34, 35, 36, 37}, AUDIENCE_BLUE_X, AUDIENCE_BLUE_Y, AUDIENCE_BLUE_YAW_DEG);
        addCluster(new int[]{38, 39, 40, 41}, FAR_RED_X, FAR_RED_Y, FAR_RED_YAW_DEG);
        addCluster(new int[]{42, 43, 44, 45}, FAR_BLUE_X, FAR_BLUE_Y, FAR_BLUE_YAW_DEG);
    }

    /**
     * Lays four tags out along a line through the cluster center, seen from
     * below: the row runs perpendicular to the yaw direction.
     */
    public static void addCluster(int[] ids, double centerX, double centerY, double yawDeg) {
        if (ids.length != 4) {
            throw new IllegalArgumentException("A BIOBUZZ cluster has exactly four tags");
        }

        double yaw = Math.toRadians(yawDeg);
        // Unit vector along the tag row (perpendicular to the facing direction).
        double alongX = -Math.sin(yaw);
        double alongY = Math.cos(yaw);

        // Three gaps between four tags; the outer two are equal per the manual.
        double g1 = CLUSTER_SPACING_OUTER_IN;
        double g2 = CLUSTER_SPACING_MIDDLE_IN;
        double g3 = CLUSTER_SPACING_OUTER_IN;

        double span = g1 + g2 + g3;
        double[] offsets = new double[4];
        offsets[0] = -span / 2.0;
        offsets[1] = offsets[0] + g1;
        offsets[2] = offsets[1] + g2;
        offsets[3] = offsets[2] + g3;

        for (int i = 0; i < 4; i++) {
            double offset = offsets[i];
            TAGS.put(ids[i], new Tag(ids[i],
                    centerX + alongX * offset,
                    centerY + alongY * offset,
                    yaw));
        }
    }

    /** Registers (or replaces) a single tag. Yaw is in degrees. */
    public static void add(int id, double xIn, double yIn, double yawDeg) {
        TAGS.put(id, new Tag(id, xIn, yIn, Math.toRadians(yawDeg)));
    }

    /** Runtime override, useful while calibrating from telemetry. */
    public static void override(int id, double xIn, double yIn, double yawDeg) {
        add(id, xIn, yIn, yawDeg);
    }

    /** Returns the tag or null when the ID is not part of this field map. */
    public static Tag get(int id) {
        return TAGS.get(id);
    }

    public static boolean contains(int id) {
        return TAGS.containsKey(id);
    }

    public static int size() {
        return TAGS.size();
    }

    public static void clear() {
        TAGS.clear();
    }

    private AprilTagMap() {
    }
}
