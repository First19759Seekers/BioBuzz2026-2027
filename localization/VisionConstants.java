package org.firstinspires.ftc.teamcode.localization;

/**
 * Every knob of the vision / fusion pipeline lives here.
 *
 * The values are plain (non-final) statics on purpose: you can tweak them live
 * from a debug OpMode without a full rebuild of the pipeline logic. Treat all
 * numbers as "starting points that a real robot must verify on the field".
 */
public class VisionConstants {

    // ------------------------------------------------------------------
    // Camera
    // ------------------------------------------------------------------

    /** Hardware-map name of the Limelight 3A. Must match your robot config. */
    public static final String LIMELIGHT_NAME = "limelight";

    /** How fast the SDK polls the camera. 100 Hz matches a 50-100 fps pipeline. */
    public static int POLL_RATE_HZ = 100;

    /** AprilTag pipeline index configured in the Limelight web UI. */
    public static final int PIPELINE_INDEX = 0;

    /** Set false if you switch pipelines by hand and do not want us to touch it. */
    public static boolean SWITCH_PIPELINE_ON_START = true;

    // ------------------------------------------------------------------
    // Camera-to-robot geometry
    // ------------------------------------------------------------------

    /**
     * Sign convention of the lateral axis of Limelight's "target pose in robot
     * space". +1 means the reported y grows to the robot's LEFT.
     *
     * How to verify: put a tag to the robot's left in front of the camera.
     * If the telemetry shows a negative lateral value, flip this to -1.
     */
    public static double ROBOT_SPACE_LEFT_SIGN = 1.0;

    /**
     * Extra yaw applied to every tag before it is used for a heading fix.
     * Only needed if your tag map uses a different facing convention than the
     * camera reports; leave at 0 for a standard map.
     */
    public static double TAG_YAW_OFFSET_DEG = 0.0;

    /**
     * Camera forward offset from the robot's rotation center, in inches.
     * Positive = camera is ahead of the center. Used to convert the
     * camera-based fix into a robot-center pose.
     */
    public static double CAMERA_FORWARD_OFFSET_IN = 0.0;

    /** Camera lateral offset from the robot's rotation center, +left, inches. */
    public static double CAMERA_LEFT_OFFSET_IN = 0.0;

    // ------------------------------------------------------------------
    // Tag quality filters
    // ------------------------------------------------------------------

    /** Tags covering less of the image than this are ignored (too far / blurry). */
    public static double MIN_TAG_AREA_FRACTION = 0.0015;

    /** Hard distance cut in inches, anything farther is not trusted. */
    public static double MAX_TAG_DISTANCE_IN = 220.0;

    /** If two tags disagree by more than this, the whole frame is rejected. */
    public static double MAX_TAG_SPREAD_IN = 18.0;

    // ------------------------------------------------------------------
    // Fusion weights
    // ------------------------------------------------------------------

    /** Correction fraction applied per frame with a single visible tag. */
    public static double TRANSLATION_GAIN_ONE_TAG = 0.35;

    /** Correction fraction applied per frame with two or more tags. */
    public static double TRANSLATION_GAIN_MULTI_TAG = 0.65;

    /**
     * Heading is primarily owned by the IMU, so tags only nudge it.
     * Set to 0 to let the gyro own heading completely.
     */
    public static double HEADING_GAIN = 0.04;

    /**
     * BIOBUZZ tags are mounted on the BOTTOM of the hives and face down at the
     * mat, so a robot camera sees them at a steep angle. The measured tag yaw
     * then describes the camera's tilt, not the robot's heading, and using it
     * would slowly rotate the pose toward a wrong heading. Position still comes
     * out correct, so heading is left entirely to the IMU.
     *
     * Only set this to true for a game whose tags face the robot horizontally.
     */
    public static boolean USE_TAG_HEADING = false;

    /**
     * Only correct heading when the tags and the gyro disagree by less than
     * this. A larger disagreement is nearly always a wrong tag facing
     * convention, and following it would spin the robot to a wrong heading.
     */
    public static double HEADING_FIX_MAX_ERROR_DEG = 8.0;

    /** Cap on a single-frame heading correction, protects against yaw glitches. */
    public static double MAX_HEADING_CORRECTION_DEG = 1.0;

    // ------------------------------------------------------------------
    // Outlier gates (a fix must be plausible before it is fused)
    // ------------------------------------------------------------------

    /** Reject a fix whose position disagrees with the prediction by more than this. */
    public static double MAX_TRANSLATION_RESIDUAL_IN = 24.0;

    /** Reject a fix whose heading disagrees with the prediction by more than this. */
    public static double MAX_HEADING_RESIDUAL_DEG = 30.0;

    // ------------------------------------------------------------------
    // Timing
    // ------------------------------------------------------------------

    /** Vision is considered "stale" after this long without a frame. */
    public static double VISION_TIMEOUT_MS = 400.0;

    /** How far the pose history is extrapolated to reach a capture timestamp. */
    public static double MAX_EXTRAPOLATION_MS = 120.0;

    /** Length of the pose history ring used for latency compensation. */
    public static double HISTORY_LENGTH_MS = 2000.0;

    /** Vision is "converged" when the residual stays under this for a while. */
    public static double CONVERGED_RESIDUAL_IN = 4.0;

    private VisionConstants() {
    }
}
