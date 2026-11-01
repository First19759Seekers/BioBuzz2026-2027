package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.PoseFollower;
import org.firstinspires.ftc.teamcode.drive.RobotHardware;
import org.firstinspires.ftc.teamcode.localization.FieldConstants;
import org.firstinspires.ftc.teamcode.localization.FieldLocalizer;
import org.firstinspires.ftc.teamcode.localization.FieldPose;

import java.util.ArrayList;
import java.util.List;

/**
 * Autonomous routine driven entirely by the field localization system.
 *
 * The program never assumes it knows where it started: during init the camera
 * keeps resolving tags, the first fix replaces the configured start pose, and
 * every later waypoint is executed against the fused pose. That means a robot
 * that is placed 6 inches off still runs the same route, and a robot that slips
 * on the way to a pickup re-acquires the correct spot from the tags.
 *
 * Subclasses only choose the alliance; the route itself is defined once in
 * {@link #buildMission()} using red-side coordinates.
 */
public abstract class FieldAutoBase extends LinearOpMode {

    protected RobotHardware hardware;
    protected MecanumDrive drive;
    protected FieldLocalizer localizer;
    protected PoseFollower follower;

    private final List<AutoStep> mission = new ArrayList<>();
    private int stepIndex;
    private AutoStep activeStep;
    private double lastLoopSeconds;
    private final ElapsedTime clock = new ElapsedTime();

    protected abstract boolean isRedAlliance();

    /** Converts a red-side pose into this alliance's pose. */
    protected FieldPose pose(FieldPose redSidePose) {
        return isRedAlliance() ? redSidePose : redSidePose.mirrorAcrossField();
    }

    protected void add(AutoStep step) {
        mission.add(step);
    }

    // ------------------------------------------------------------------
    // Mission definition
    // ------------------------------------------------------------------

    protected void buildMission() {
        // 1. Make sure the start pose really came from the field, not from a guess.
        add(new AutoSteps.VisionSettle(drive, localizer, 2.0));

        // 2. Preload: drive to the scoring spot and shoot.
        add(new AutoSteps.DriveTo("preload", follower, localizer, drive,
                pose(AutoPoses.SCORE_PRELOAD), 5.0));
        addShootSequence(1.2);

        // 3. Cycle one: intake running while driving, so the pickup is not a stop.
        add(new AutoSteps.Action("intake on", () -> setIntakePower(1.0)));
        add(new AutoSteps.DriveTo("to pickup 1", follower, localizer, drive,
                pose(AutoPoses.PICKUP_ONE), 6.0));
        add(new AutoSteps.Wait("collect", drive, 0.6));
        add(new AutoSteps.Action("intake off", () -> setIntakePower(0.0)));

        add(new AutoSteps.DriveTo("score cycle", follower, localizer, drive,
                pose(AutoPoses.SCORE_CYCLE), 6.0));
        addShootSequence(1.0);

        // 4. Cycle two, re-localizing before the shot.
        add(new AutoSteps.Action("intake on", () -> setIntakePower(1.0)));
        add(new AutoSteps.DriveTo("to pickup 2", follower, localizer, drive,
                pose(AutoPoses.PICKUP_TWO), 6.0));
        add(new AutoSteps.Wait("collect", drive, 0.6));
        add(new AutoSteps.Action("intake off", () -> setIntakePower(0.0)));

        add(new AutoSteps.DriveTo("final score", follower, localizer, drive,
                pose(AutoPoses.SCORE_FINAL), 6.0));
        add(new AutoSteps.VisionSettle(drive, localizer, 1.0));
        addShootSequence(1.0);

        // 5. Park.
        add(new AutoSteps.DriveTo("park", follower, localizer, drive,
                pose(AutoPoses.PARK), 5.0));
        add(new AutoSteps.Action("all stop", this::stopMechanisms));
    }

    /** Spin up, feed, and stop. Written once so both cycles behave identically. */
    private void addShootSequence(double spinUpSeconds) {
        add(new AutoSteps.Action("shooter on", () -> setShooterPower(0.85)));
        add(new AutoSteps.Wait("spin up", drive, spinUpSeconds));
        add(new AutoSteps.Action("feed", this::openFeeder));
        add(new AutoSteps.Wait("feed", drive, 0.45));
        add(new AutoSteps.Action("stop feed", () -> {
            closeFeeder();
            setShooterPower(0.0);
        }));
    }

    // ------------------------------------------------------------------
    // Mechanism hooks: no-ops when the device is not in the configuration
    // ------------------------------------------------------------------

    protected void setShooterPower(double power) {
        if (hardware != null && hardware.shooter != null) {
            hardware.shooter.setPower(power);
        }
    }

    protected void setIntakePower(double power) {
        if (hardware != null && hardware.intake != null) {
            hardware.intake.setPower(power);
        }
    }

    protected void openFeeder() {
        if (hardware != null && hardware.feeder != null) {
            hardware.feeder.setPosition(0.35);
        }
    }

    protected void closeFeeder() {
        if (hardware != null && hardware.feeder != null) {
            hardware.feeder.setPosition(0.0);
        }
    }

    protected void stopMechanisms() {
        setShooterPower(0.0);
        setIntakePower(0.0);
        closeFeeder();
    }

    // ------------------------------------------------------------------
    // OpMode lifecycle
    // ------------------------------------------------------------------

    @Override
    public void runOpMode() {
        hardware = new RobotHardware(hardwareMap);
        drive = new MecanumDrive(hardware);
        localizer = new FieldLocalizer(drive, hardwareMap);
        follower = new PoseFollower(drive);

        buildMission();

        // Seed with the configured start pose; the first tag fix overwrites it.
        localizer.setPose(pose(AutoPoses.START));

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("FIELD LOCALIZATION AUTO");
        telemetry.addLine("Alliance: " + (isRedAlliance() ? "RED" : "BLUE"));
        telemetry.addLine("Keep the robot still during init.");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            localizer.update();
            telemetry.addData("pose", localizer.getPose());
            telemetry.addData("odom", localizer.getOdometryPose());
            telemetry.addData("vision", localizer.getStatusLine());
            telemetry.addData("camera", localizer.getVision().getStatusLine());
            telemetry.update();
        }

        if (isStopRequested()) {
            return;
        }

        // Re-zero dead reckoning on the freshly localized pose.
        localizer.anchor();
        clock.reset();
        lastLoopSeconds = 0.0;

        while (opModeIsActive()) {
            double now = clock.seconds();
            double dt = Math.min(now - lastLoopSeconds, 0.1);
            lastLoopSeconds = now;

            localizer.update();
            advanceMission(dt);

            FieldPose current = localizer.getPose();
            telemetry.addData("step", stepIndex + "/" + mission.size() + " "
                    + (activeStep != null ? activeStep.name() : "done"));
            telemetry.addData("pose", current);
            telemetry.addData("vision", localizer.getStatusLine());
            if (!FieldConstants.isInsideField(current)) {
                telemetry.addLine("WARNING: pose outside the field");
            }
            telemetry.update();
        }

        drive.stop();
        hardware.stopAll();
        localizer.getVision().stop();
    }

    private void advanceMission(double dt) {
        if (activeStep == null) {
            if (stepIndex >= mission.size()) {
                drive.stop();
                return;
            }
            activeStep = mission.get(stepIndex);
            activeStep.begin();
        }

        if (activeStep.update(dt)) {
            activeStep.end();
            stepIndex++;
            activeStep = null;
        }
    }
}
