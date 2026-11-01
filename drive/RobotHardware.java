package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Single place where every device is looked up.
 *
 * Every device is resolved through a list of accepted names, so the same code
 * runs whether the robot configuration was built with "fl/fr/bl/br" or the much
 * more common "leftFrontMotor/rightFrontMotor/...". The IMU likewise accepts a
 * modern hub IMU or the legacy external BNO055.
 *
 * Only the four drive motors are required. Everything else comes back as null
 * when the configuration does not contain it, which is what lets the same
 * autonomous OpMode run on a bare chassis and on the finished robot.
 */
@SuppressWarnings("deprecation")
public class RobotHardware {

    public final DcMotor frontLeft;
    public final DcMotor frontRight;
    public final DcMotor backLeft;
    public final DcMotor backRight;

    /** Modern hub IMU, null when the config only has a BNO055. */
    private final IMU hubImu;
    /** Legacy external BNO055, null when not present. */
    private final BNO055IMU bnoImu;

    /** Optional mechanisms. */
    public final DcMotor shooter;
    public final DcMotor intake;
    public final Servo feeder;

    public RobotHardware(HardwareMap hardwareMap) {
        frontLeft = requireMotor(hardwareMap, DriveConstants.LEFT_SIDE_REVERSED,
                DriveConstants.LEFT_FRONT_NAME, "leftFrontMotor", "leftfront", "fl", "lf");
        backLeft = requireMotor(hardwareMap, DriveConstants.LEFT_SIDE_REVERSED,
                DriveConstants.LEFT_BACK_NAME, "leftBackMotor", "leftback", "bl", "lb");
        frontRight = requireMotor(hardwareMap, false,
                DriveConstants.RIGHT_FRONT_NAME, "rightFrontMotor", "rightfront", "fr", "rf");
        backRight = requireMotor(hardwareMap, false,
                DriveConstants.RIGHT_BACK_NAME, "rightBackMotor", "rightback", "br", "rb");

        hubImu = tryHubImu(hardwareMap);
        bnoImu = hubImu == null ? tryBno055(hardwareMap) : null;

        shooter = optionalMotor(hardwareMap, DriveConstants.SHOOTER_MOTOR_NAME);
        intake = optionalMotor(hardwareMap, DriveConstants.INTAKE_MOTOR_NAME);
        feeder = optionalServo(hardwareMap, DriveConstants.FEEDER_SERVO_NAME);
    }

    private static DcMotor requireMotor(HardwareMap hardwareMap, boolean reversed, String... candidates) {
        DcMotor motor = null;
        for (String name : candidates) {
            try {
                motor = hardwareMap.get(DcMotor.class, name);
                break;
            } catch (RuntimeException ignored) {
                // Try the next accepted name.
            }
        }
        if (motor == null) {
            StringBuilder message = new StringBuilder("No drive motor found. Tried:");
            for (String name : candidates) {
                message.append(' ').append(name);
            }
            message.append(". Rename the motor in the Driver Station config or edit DriveConstants.");
            throw new IllegalStateException(message.toString());
        }

        motor.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0.0);
        return motor;
    }

    private static DcMotor optionalMotor(HardwareMap hardwareMap, String name) {
        try {
            DcMotor motor = hardwareMap.get(DcMotor.class, name);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setPower(0.0);
            return motor;
        } catch (RuntimeException e) {
            return null;
        }
    }

    private static Servo optionalServo(HardwareMap hardwareMap, String name) {
        try {
            return hardwareMap.get(Servo.class, name);
        } catch (RuntimeException e) {
            return null;
        }
    }

    private static IMU tryHubImu(HardwareMap hardwareMap) {
        for (String name : new String[]{DriveConstants.IMU_NAME, "IMU", "imu1"}) {
            try {
                IMU localImu = hardwareMap.get(IMU.class, name);
                localImu.initialize(new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
                localImu.resetYaw();
                return localImu;
            } catch (RuntimeException ignored) {
                // Try the next accepted name.
            }
        }
        return null;
    }

    private static BNO055IMU tryBno055(HardwareMap hardwareMap) {
        for (String name : new String[]{DriveConstants.IMU_NAME, "IMU", "bno055"}) {
            try {
                BNO055IMU legacy = hardwareMap.get(BNO055IMU.class, name);
                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
                legacy.initialize(parameters);
                return legacy;
            } catch (RuntimeException ignored) {
                // Try the next accepted name.
            }
        }
        return null;
    }

    public boolean hasImu() {
        return hubImu != null || bnoImu != null;
    }

    /**
     * Raw yaw of whichever IMU is present, in radians, or NaN when there is
     * none. Sign and zero point are handled by the odometry layer.
     */
    public double readYawRadians() {
        if (hubImu != null) {
            return hubImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }
        if (bnoImu != null) {
            Orientation orientation = bnoImu.getAngularOrientation(
                    AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            return orientation.firstAngle;
        }
        return Double.NaN;
    }

    public void resetImuYaw() {
        if (hubImu != null) {
            hubImu.resetYaw();
        }
    }

    public boolean hasShooter() {
        return shooter != null;
    }

    public boolean hasIntake() {
        return intake != null;
    }

    public boolean hasFeeder() {
        return feeder != null;
    }

    public void stopAll() {
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
        if (shooter != null) {
            shooter.setPower(0.0);
        }
        if (intake != null) {
            intake.setPower(0.0);
        }
    }
}
