package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Teleop", group="Linear OpMode")
public class TeleopShanghai2 extends LinearOpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight,shooter;
    IMU imu;

    boolean headless = false;
    boolean lastX = false;
    static final double DEADZONE = 0.05;
    static final double SHOOTER_POWER = 0.8;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "leftfront");
        frontRight = hardwareMap.get(DcMotor.class, "rightfront");
        backLeft = hardwareMap.get(DcMotor.class, "leftback");
        backRight = hardwareMap.get(DcMotor.class, "rightback");
        shooter = hardwareMap.get(DcMotor.class, "Shooter");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);
        imu.resetYaw();

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();
        imu.resetYaw();

        while (opModeIsActive()) {
            // 按X切换无头模式
            if (gamepad1.x && !lastX) {
                headless = !headless;
                if (headless) imu.resetYaw();
            }
            lastX = gamepad1.x;

            double lx = Math.abs(gamepad1.left_stick_x) > DEADZONE ? gamepad1.left_stick_x : 0;
            double ly = Math.abs(gamepad1.left_stick_y) > DEADZONE ? gamepad1.left_stick_y : 0;
            double rx = Math.abs(gamepad1.right_stick_x) > DEADZONE ? gamepad1.right_stick_x : 0;

            double rawForward = -ly;
            double rawStrafe  = 0.7 * lx;
            double r_manual   = 0.9 * rx;
            double y, x;

            if (headless) {
                double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double cos = Math.cos(heading);
                double sin = Math.sin(heading);
                double robotX = -rawStrafe * cos - rawForward * sin;
                double robotY = -rawStrafe * sin + rawForward * cos;
                y = robotY;
                x = -robotX;
            } else {
                y = rawForward;
                x = rawStrafe;
            }

            double r = r_manual;

            double speed = gamepad1.right_bumper ? 0.5 : 0.8;
            double fl = y + x + r;
            double fr = y - x - r;
            double bl = y - x + r;
            double br = y + x - r;

            double maxVal = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
            if (maxVal > 1.0) {
                fl /= maxVal; fr /= maxVal; bl /= maxVal; br /= maxVal;
            }

            frontLeft.setPower(fl * speed);
            frontRight.setPower(fr * speed);
            backLeft.setPower(bl * speed);
            backRight.setPower(br * speed);

            // ========== Shooter: gamepad2 right trigger ==========
            if(gamepad2.right_trigger > 0.1){
                shooter.setPower(SHOOTER_POWER);
            }else{
                shooter.setPower(0);
            }

            telemetry.addData("Headless Mode", headless ? "ON" : "OFF");
            telemetry.addData("Shooter", gamepad2.right_trigger>0.1 ? "RUNNING 50%" : "STOP");
            telemetry.update();
        }
    }
}
