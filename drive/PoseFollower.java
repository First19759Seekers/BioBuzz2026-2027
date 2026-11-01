package org.firstinspires.ftc.teamcode.drive;

import org.firstinspires.ftc.teamcode.localization.FieldPose;
import org.firstinspires.ftc.teamcode.util.PIDController;

/**
 * Field-frame "go to this pose" controller used by the autonomous program.
 *
 * It is deliberately small: a trapezoidal speed profile for translation plus a
 * PID on heading. That is enough to drive waypoint-to-waypoint while the
 * localization layer keeps feeding a corrected pose, and it has no dependency on
 * a path-following library.
 *
 * Because the input pose is the fused (vision corrected) pose, a waypoint that
 * was missed by 3 inches due to wheel slip gets driven back on target by the
 * next vision fix instead of compounding.
 */
public class PoseFollower {

    private final MecanumDrive drive;
    private final PIDController headingController;

    private FieldPose target;
    private double previousSpeed;

    public PoseFollower(MecanumDrive drive) {
        this.drive = drive;
        this.headingController = new PIDController(
                DriveConstants.PATH_HEADING_KP,
                DriveConstants.PATH_HEADING_KI,
                DriveConstants.PATH_HEADING_KD);
        this.headingController.setOutputLimit(DriveConstants.PATH_MAX_ANGULAR_SPEED_RAD_PER_SEC);
        this.headingController.setIntegralLimit(0.5);
    }

    public void setTarget(FieldPose target) {
        this.target = target;
        this.previousSpeed = 0.0;
        headingController.reset();
    }

    public void clearTarget() {
        this.target = null;
        this.previousSpeed = 0.0;
    }

    public boolean hasTarget() {
        return target != null;
    }

    public FieldPose getTarget() {
        return target;
    }

    public double distanceToTarget(FieldPose current) {
        return target == null ? 0.0 : current.distanceTo(target);
    }

    public double headingError(FieldPose current) {
        return target == null ? 0.0 : current.headingError(target.heading);
    }

    public boolean atTarget(FieldPose current) {
        if (target == null) {
            return true;
        }
        boolean positionOk = current.distanceTo(target) <= DriveConstants.PATH_POSITION_TOLERANCE_IN;
        boolean headingOk = Math.abs(Math.toDegrees(current.headingError(target.heading)))
                <= DriveConstants.PATH_HEADING_TOLERANCE_DEG;
        return positionOk && headingOk;
    }

    /** Drives toward the target. Call once per loop; stop() when finished. */
    public void update(FieldPose current, double dt) {
        if (target == null) {
            drive.stop();
            return;
        }

        double dx = target.x - current.x;
        double dy = target.y - current.y;
        double distance = Math.hypot(dx, dy);

        double headingError = current.headingError(target.heading);
        double omega = headingController.calculate(headingError, dt);

        if (distance < 0.05) {
            drive.driveFieldRelative(0.0, 0.0, omega, current.heading);
            previousSpeed = 0.0;
            return;
        }

        // Ramp down on approach, ramp up on departure, never exceed the cap.
        double decelCap = Math.sqrt(2.0 * DriveConstants.PATH_MAX_ACCEL_IN_PER_SEC2 * distance) + 2.0;
        double desiredSpeed = Math.min(DriveConstants.PATH_MAX_SPEED_IN_PER_SEC, decelCap);
        desiredSpeed = Math.min(desiredSpeed, 4.0 * distance);

        double maxDelta = DriveConstants.PATH_MAX_ACCEL_IN_PER_SEC2 * Math.max(dt, 0.0);
        double speed = previousSpeed + clamp(desiredSpeed - previousSpeed, -maxDelta, maxDelta);
        previousSpeed = speed;

        double vx = dx / distance * speed;
        double vy = dy / distance * speed;

        drive.driveFieldRelative(vx, vy, omega, current.heading);
    }

    public void stop() {
        drive.stop();
        clearTarget();
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
