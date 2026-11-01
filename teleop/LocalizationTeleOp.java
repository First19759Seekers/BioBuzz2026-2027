package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.RobotHardware;
import org.firstinspires.ftc.teamcode.localization.AprilTagMap;
import org.firstinspires.ftc.teamcode.localization.FieldConstants;
import org.firstinspires.ftc.teamcode.localization.FieldLocalizer;
import org.firstinspires.ftc.teamcode.localization.FieldPose;

/**
 * Verification OpMode for the localization system. Run this before trusting auto.
 *
 * What to check, in order:
 *   1. Push the robot forward: X must grow. Push it left: Y must grow.
 *      Rotate it CCW: heading must grow. If any of these fail, adjust the signs
 *      in DriveConstants (encoder signs / IMU_HEADING_SIGN) before going further.
 *   2. Put a tag in front of the camera: the "vision" pose must be within a few
 *      inches of the true position, and the residual must be small. A constant
 *      offset means a wrong number in AprilTagMap, not a fusion problem.
 *   3. Drive around: the fused pose must stay on the field drawing while the
 *      odometry pose slowly diverges. That divergence being corrected is exactly
 *      what the camera is for.
 *
 * Controls: A anchors dead reckoning to the fused pose, B resets the heading to
 * zero, X toggles field-centric drive, Y prints the tag map to telemetry.
 */
@TeleOp(name = "Localization TeleOp", group = "Diagnostics")
public class LocalizationTeleOp extends LinearOpMode {

    private MecanumDrive drive;
    private RobotHardware hardware;
    private FieldLocalizer localizer;

    private boolean fieldCentric = true;
    private final ElapsedTime clock = new ElapsedTime();
    private double lastLoopSeconds;

    @Override
    public void runOpMode() {
        hardware = new RobotHardware(hardwareMap);
        drive = new MecanumDrive(hardware);
        localizer = new FieldLocalizer(drive, hardwareMap);
        localizer.setPose(FieldPose.ZERO);

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Localization TeleOp");
        telemetry.addLine("A anchor | B zero heading | X field/robot | Y tag map");
        telemetry.update();

        waitForStart();

        clock.reset();
        lastLoopSeconds = 0.0;

        while (opModeIsActive()) {
            double now = clock.seconds();
            double dt = Math.min(now - lastLoopSeconds, 0.1);
            lastLoopSeconds = now;

            localizer.update();

            double speed = gamepad1.right_trigger > 0.2 ? 1.0 : 0.55;
            double forward = -gamepad1.left_stick_y * DriveConstants.MAX_SPEED_IN_PER_SEC * speed;
            double left = -gamepad1.left_stick_x * DriveConstants.MAX_SPEED_IN_PER_SEC * speed;
            double turn = -gamepad1.right_stick_x * DriveConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC * speed;

            if (fieldCentric) {
                drive.driveFieldRelative(forward, left, turn, localizer.getPose().heading);
            } else {
                drive.driveRobotRelative(forward, left, turn);
            }

            if (gamepad1.a) {
                localizer.anchor();
            }
            if (gamepad1.b) {
                localizer.setPose(localizer.getPose().withHeading(0.0));
            }
            if (gamepad1.x) {
                fieldCentric = !fieldCentric;
            }

            FieldPose fused = localizer.getPose();
            FieldPose odom = localizer.getOdometryPose();

            telemetry.addData("mode", fieldCentric ? "field centric" : "robot centric");
            telemetry.addData("pose", fused);
            telemetry.addData("odom", odom);
            telemetry.addData("drift", String.format("%.1f in", fused.distanceTo(odom)));
            telemetry.addData("vision", localizer.getStatusLine());
            telemetry.addData("camera", localizer.getVision().getStatusLine());
            telemetry.addData("residual", String.format("%.1f in / %.1f deg",
                    localizer.getResidualIn(), localizer.getHeadingResidualDeg()));
            telemetry.addData("frames", localizer.getAcceptedFrames() + " ok / "
                    + localizer.getRejectedFrames() + " rejected");
            telemetry.addData("field", FieldConstants.isInsideField(fused) ? "ok" : "OUT OF BOUNDS");

            if (gamepad1.y) {
                for (int i = 0; i < 32; i++) {
                    if (AprilTagMap.contains(i)) {
                        telemetry.addLine(AprilTagMap.get(i).toString());
                    }
                }
            }

            telemetry.update();
        }

        drive.stop();
        hardware.stopAll();
        localizer.getVision().stop();
    }
}
