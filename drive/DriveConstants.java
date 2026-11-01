package org.firstinspires.ftc.teamcode.drive;

/**
 * Drivetrain wiring and robot geometry.
 *
 * Names must match the robot configuration on the Driver Station. If a device is
 * missing the code degrades instead of crashing (see {@link RobotHardware}),
 * so a bare chassis can run the localization stack before the mechanisms are done.
 */
public class DriveConstants {

    // ------------------------------------------------------------------
    // Hardware map names
    // ------------------------------------------------------------------

    public static final String LEFT_FRONT_NAME = "leftfront";
    public static final String RIGHT_FRONT_NAME = "rightfront";
    public static final String LEFT_BACK_NAME = "leftback";
    public static final String RIGHT_BACK_NAME = "rightback";

    public static final String IMU_NAME = "imu";

    // Optional mechanisms, used by the auto OpMode when present, skipped when not.
    public static final String SHOOTER_MOTOR_NAME = "shooter";
    public static final String INTAKE_MOTOR_NAME = "intake";
    public static final String FEEDER_SERVO_NAME = "feeder";

    // ------------------------------------------------------------------
    // Motor polarity
    // ------------------------------------------------------------------

    /** Left side motors are reversed so that positive power always means forward. */
    public static boolean LEFT_SIDE_REVERSED = true;

    // ------------------------------------------------------------------
    // Geometry
    // ------------------------------------------------------------------

    /** goBILDA 96 mm mecanum wheel. */
    public static double WHEEL_DIAMETER_IN = 3.78;

    /** Encoder counts per motor revolution (goBILDA 5202 312 RPM = 537.7). */
    public static double ENCODER_TICKS_PER_REV = 537.7;

    public static double TICKS_PER_INCH = ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_IN * Math.PI);

    /**
     * Distance between the left and right wheel contact patches, inches.
     * Chassis is 42 x 45 cm (16.5 x 17.7 in); this is measured across the wheel
     * contact patches, not the frame, so re-measure it with a tape measure.
     */
    public static double TRACK_WIDTH_IN = 15.5;

    /** Distance between front and back wheel contact patches, inches. */
    public static double WHEEL_BASE_IN = 14.5;

    // ------------------------------------------------------------------
    // Per-wheel encoder signs
    // ------------------------------------------------------------------
    // The SDK already inverts the reported position for a motor whose direction
    // is REVERSE, but that behaviour changed across releases. These multipliers
    // make it explicit: push the robot straight forward 24 in and every wheel
    // count should read positive. Flip a sign here if one wheel reads backwards.

    public static double ENCODER_SIGN_FL = 1.0;
    public static double ENCODER_SIGN_FR = 1.0;
    public static double ENCODER_SIGN_BL = 1.0;
    public static double ENCODER_SIGN_BR = 1.0;

    // ------------------------------------------------------------------
    // IMU
    // ------------------------------------------------------------------

    /**
     * +1 if the hub yaw grows counter-clockwise, -1 if it grows clockwise.
     * Verify with the Localization TeleOp: rotate the robot CCW by hand, the
     * reported heading must increase. Flip this constant if it decreases.
     */
    public static double IMU_HEADING_SIGN = 1.0;

    // ------------------------------------------------------------------
    // Limits
    // ------------------------------------------------------------------

    /** Measured top speed of the chassis, inches per second. */
    public static double MAX_SPEED_IN_PER_SEC = 75.0;

    /** Top commanded rotation rate, radians per second. */
    public static double MAX_ANGULAR_SPEED_RAD_PER_SEC = 5.0;

    /**
     * Empirical multiplier on the turning lever arm. Mecanum wheels slip
     * sideways while rotating, so the geometric value under-turns on most
     * robots; 1.2-1.4 is typical. The heading loop closes the remaining error,
     * so this only affects how snappy a rotation feels.
     */
    public static double ROTATION_SCALE = 1.25;

    // ------------------------------------------------------------------
    // Motor control
    // ------------------------------------------------------------------

    /** Feed-forward gain that converts an inch/second command into motor power. */
    public static double POWER_PER_IN_PER_SEC = 1.0 / MAX_SPEED_IN_PER_SEC;

    /** Feed-forward gain that converts rad/second into a normalised rotation term. */
    public static double POWER_PER_RAD_PER_SEC = 1.0 / MAX_ANGULAR_SPEED_RAD_PER_SEC;

    /**
     * Exponential smoothing applied to the drive command.
     * 1.0 = passthrough (correct for autonomous, the follower already ramps),
     * 0.3-0.6 = noticeably smoother stick response in TeleOp.
     */
    public static double DRIVE_COMMAND_SMOOTHING = 1.0;

    // ------------------------------------------------------------------
    // Path following (used by the auto OpMode)
    // ------------------------------------------------------------------

    public static double PATH_MAX_SPEED_IN_PER_SEC = 45.0;
    public static double PATH_MAX_ACCEL_IN_PER_SEC2 = 60.0;
    public static double PATH_HEADING_KP = 2.2;
    public static double PATH_HEADING_KI = 0.0;
    public static double PATH_HEADING_KD = 0.05;
    public static double PATH_POSITION_TOLERANCE_IN = 1.2;
    public static double PATH_HEADING_TOLERANCE_DEG = 2.5;
    public static double PATH_MAX_ANGULAR_SPEED_RAD_PER_SEC = 3.0;

    private DriveConstants() {
    }
}
