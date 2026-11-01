package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Four-wheel mecanum drivetrain.
 *
 * Two command frames are supported:
 *   - robot relative: +forward, +left, +CCW rotation
 *   - field relative: +X, +Y of the field frame plus a heading to rotate against
 *
 * Everything is in inches/second and radians/second so it can be fed directly
 * from the localization layer without unit conversions scattered around.
 */
public class MecanumDrive {

    private final RobotHardware hardware;

    private double lastForward;
    private double lastLeft;
    private double lastOmega;

    public MecanumDrive(RobotHardware hardware) {
        this.hardware = hardware;
    }

    public RobotHardware getHardware() {
        return hardware;
    }

    /** +forward / +left in inches per second, +omega is CCW in radians per second. */
    public void driveRobotRelative(double forwardIn, double leftIn, double omegaRad) {
        double smoothing = clamp(DriveConstants.DRIVE_COMMAND_SMOOTHING, 0.0, 1.0);
        lastForward += (forwardIn - lastForward) * smoothing;
        lastLeft += (leftIn - lastLeft) * smoothing;
        lastOmega += (omegaRad - lastOmega) * smoothing;

        // Turning in place: each contact patch travels on an arc of radius
        // trackWidth / 2, so that is the lever arm that converts rad/s to in/s.
        double rotationRadius = DriveConstants.TRACK_WIDTH_IN / 2.0 * DriveConstants.ROTATION_SCALE;

        double frontLeft = lastForward - lastLeft - rotationRadius * lastOmega;
        double frontRight = lastForward + lastLeft + rotationRadius * lastOmega;
        double backLeft = lastForward + lastLeft - rotationRadius * lastOmega;
        double backRight = lastForward - lastLeft + rotationRadius * lastOmega;

        // Normalise to the achievable wheel speed so the heading is preserved
        // even when the chassis is asked to do more than it physically can.
        double maxWheel = Math.max(
                Math.max(Math.abs(frontLeft), Math.abs(frontRight)),
                Math.max(Math.abs(backLeft), Math.abs(backRight)));
        double limit = DriveConstants.MAX_SPEED_IN_PER_SEC;
        double scale = maxWheel > limit ? limit / maxWheel : 1.0;

        setPowers(frontLeft * scale, frontRight * scale, backLeft * scale, backRight * scale);
    }

    /** Field-frame command, rotated into the robot frame using the current heading. */
    public void driveFieldRelative(double xField, double yField, double omegaRad, double headingRad) {
        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);
        double forward = xField * cos + yField * sin;
        double left = -xField * sin + yField * cos;
        driveRobotRelative(forward, left, omegaRad);
    }

    private void setPowers(double frontLeftIn, double frontRightIn, double backLeftIn, double backRightIn) {
        double k = DriveConstants.POWER_PER_IN_PER_SEC;
        hardware.frontLeft.setPower(clamp(frontLeftIn * k, -1.0, 1.0));
        hardware.frontRight.setPower(clamp(frontRightIn * k, -1.0, 1.0));
        hardware.backLeft.setPower(clamp(backLeftIn * k, -1.0, 1.0));
        hardware.backRight.setPower(clamp(backRightIn * k, -1.0, 1.0));
    }

    public void stop() {
        lastForward = 0.0;
        lastLeft = 0.0;
        lastOmega = 0.0;
        hardware.frontLeft.setPower(0.0);
        hardware.frontRight.setPower(0.0);
        hardware.backLeft.setPower(0.0);
        hardware.backRight.setPower(0.0);
    }

    /** Wheel travel in inches, forward positive, in FL, FR, BL, BR order. */
    public double[] getWheelPositionsInches() {
        double ticksToInches = 1.0 / DriveConstants.TICKS_PER_INCH;
        return new double[]{
                hardware.frontLeft.getCurrentPosition() * ticksToInches * DriveConstants.ENCODER_SIGN_FL,
                hardware.frontRight.getCurrentPosition() * ticksToInches * DriveConstants.ENCODER_SIGN_FR,
                hardware.backLeft.getCurrentPosition() * ticksToInches * DriveConstants.ENCODER_SIGN_BL,
                hardware.backRight.getCurrentPosition() * ticksToInches * DriveConstants.ENCODER_SIGN_BR
        };
    }

    public void resetEncoders() {
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setMotorMode(DcMotor.RunMode mode) {
        hardware.frontLeft.setMode(mode);
        hardware.frontRight.setMode(mode);
        hardware.backLeft.setMode(mode);
        hardware.backRight.setMode(mode);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
