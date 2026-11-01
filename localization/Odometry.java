package org.firstinspires.ftc.teamcode.localization;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

/**
 * Dead-reckoning odometry from the four drive encoders plus the hub IMU.
 *
 * This is the "prediction" half of the localization: it is smooth and never
 * jumps, but it drifts. The vision layer corrects it.
 *
 * If your robot carries a dedicated tracking computer (goBILDA Pinpoint, OTOS,
 * a three-wheel pod), this is the class you replace: keep the same public API
 * and the rest of the stack keeps working unchanged.
 */
public class Odometry {

    private final MecanumDrive drive;

    private FieldPose pose = FieldPose.ZERO;
    private double[] lastWheels;
    private boolean initialized;

    /** Offset that makes the IMU report heading in the field frame. */
    private double headingOffset;

    public Odometry(MecanumDrive drive) {
        this.drive = drive;
    }

    /** Call once per loop, before anything reads {@link #getPose()}. */
    public void update() {
        double[] wheels = drive.getWheelPositionsInches();
        if (!initialized) {
            lastWheels = wheels;
            initialized = true;
        }

        double dFrontLeft = wheels[0] - lastWheels[0];
        double dFrontRight = wheels[1] - lastWheels[1];
        double dBackLeft = wheels[2] - lastWheels[2];
        double dBackRight = wheels[3] - lastWheels[3];
        lastWheels = wheels;

        double dForward = (dFrontLeft + dFrontRight + dBackLeft + dBackRight) / 4.0;
        double dLeft = (-dFrontLeft + dFrontRight + dBackLeft - dBackRight) / 4.0;

        double newHeading = readHeading();
        double dHeading;
        if (Double.isNaN(newHeading)) {
            // Wheel-only fallback: the right side leading the left side means CCW.
            double dRight = (dFrontRight + dBackRight) / 2.0;
            double dLeftSide = (dFrontLeft + dBackLeft) / 2.0;
            dHeading = (dRight - dLeftSide) / DriveConstants.TRACK_WIDTH_IN;
            newHeading = FieldPose.normalizeAngle(pose.heading + dHeading);
        } else {
            dHeading = FieldPose.normalizeAngle(newHeading - pose.heading);
        }

        // Rotate the body-frame travel into the field frame using the heading at
        // the start of the interval. At 50+ Hz the error from this approximation
        // is negligible compared with encoder quantisation.
        double cos = Math.cos(pose.heading);
        double sin = Math.sin(pose.heading);
        double dx = dForward * cos - dLeft * sin;
        double dy = dForward * sin + dLeft * cos;

        pose = new FieldPose(pose.x + dx, pose.y + dy, pose.heading + dHeading);
    }

    /** Field-frame heading straight from the IMU, or NaN when there is no IMU. */
    public double readHeading() {
        double yaw = drive.getHardware().readYawRadians();
        if (Double.isNaN(yaw)) {
            return Double.NaN;
        }
        return FieldPose.normalizeAngle(DriveConstants.IMU_HEADING_SIGN * yaw + headingOffset);
    }

    public FieldPose getPose() {
        return pose;
    }

    /** Hard reset: also re-references the gyro so heading 0 matches the new pose. */
    public void setPose(FieldPose newPose) {
        pose = newPose;
        resetHeading(newPose.heading);
    }

    public void resetHeading(double desiredHeadingRad) {
        double yaw = drive.getHardware().readYawRadians();
        if (Double.isNaN(yaw)) {
            pose = pose.withHeading(desiredHeadingRad);
            return;
        }
        headingOffset = FieldPose.normalizeAngle(desiredHeadingRad - DriveConstants.IMU_HEADING_SIGN * yaw);
        pose = pose.withHeading(desiredHeadingRad);
    }

    /** True once at least one update has run, i.e. wheel deltas are meaningful. */
    public boolean isInitialized() {
        return initialized;
    }
}
