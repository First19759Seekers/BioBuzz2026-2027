package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.PoseFollower;
import org.firstinspires.ftc.teamcode.localization.FieldLocalizer;
import org.firstinspires.ftc.teamcode.localization.FieldPose;

/**
 * Reusable building blocks for autonomous routines.
 *
 * Every step runs against the fused localization pose, so no step needs to know
 * whether the robot is being driven by encoders or corrected by a tag at that
 * moment.
 */
public class AutoSteps {

    private AutoSteps() {
    }

    /** Drives to a field pose and waits for the follower to settle. */
    public static class DriveTo implements AutoStep {
        private final String label;
        private final PoseFollower follower;
        private final FieldLocalizer localizer;
        private final MecanumDrive drive;
        private final FieldPose target;
        private final double timeoutSeconds;

        private final ElapsedTime timer = new ElapsedTime();
        private boolean timedOut;

        public DriveTo(String label, PoseFollower follower, FieldLocalizer localizer,
                       MecanumDrive drive, FieldPose target, double timeoutSeconds) {
            this.label = label;
            this.follower = follower;
            this.localizer = localizer;
            this.drive = drive;
            this.target = target;
            this.timeoutSeconds = timeoutSeconds;
        }

        @Override
        public String name() {
            return label + " -> " + target;
        }

        @Override
        public void begin() {
            follower.setTarget(target);
            timer.reset();
            timedOut = false;
        }

        @Override
        public boolean update(double dtSeconds) {
            FieldPose current = localizer.getPose();
            follower.update(current, dtSeconds);

            if (follower.atTarget(current)) {
                return true;
            }
            if (timer.seconds() > timeoutSeconds) {
                timedOut = true;
                return true;
            }
            return false;
        }

        @Override
        public void end() {
            drive.stop();
            follower.clearTarget();
        }

        public boolean timedOut() {
            return timedOut;
        }
    }

    /** Fire-and-forget action, used to poke a mechanism between drives. */
    public static class Action implements AutoStep {
        private final String label;
        private final Runnable action;

        public Action(String label, Runnable action) {
            this.label = label;
            this.action = action;
        }

        @Override
        public String name() {
            return label;
        }

        @Override
        public void begin() {
            action.run();
        }

        @Override
        public boolean update(double dtSeconds) {
            return true;
        }

        @Override
        public void end() {
        }
    }

    /** Plain timed pause, with the robot held still. */
    public static class Wait implements AutoStep {
        private final String label;
        private final MecanumDrive drive;
        private final double seconds;
        private final ElapsedTime timer = new ElapsedTime();

        public Wait(String label, MecanumDrive drive, double seconds) {
            this.label = label;
            this.drive = drive;
            this.seconds = seconds;
        }

        @Override
        public String name() {
            return label;
        }

        @Override
        public void begin() {
            drive.stop();
            timer.reset();
        }

        @Override
        public boolean update(double dtSeconds) {
            return timer.seconds() >= seconds;
        }

        @Override
        public void end() {
            drive.stop();
        }
    }

    /**
     * Stands still until the localization has a fresh, self-consistent tag fix,
     * or until the timeout expires. Used before a critical shot so the routine
     * never aims with stale coordinates.
     */
    public static class VisionSettle implements AutoStep {
        private final MecanumDrive drive;
        private final FieldLocalizer localizer;
        private final double timeoutSeconds;
        private final ElapsedTime timer = new ElapsedTime();
        private boolean locked;

        public VisionSettle(MecanumDrive drive, FieldLocalizer localizer, double timeoutSeconds) {
            this.drive = drive;
            this.localizer = localizer;
            this.timeoutSeconds = timeoutSeconds;
        }

        @Override
        public String name() {
            return "vision settle";
        }

        @Override
        public void begin() {
            drive.stop();
            timer.reset();
            locked = false;
        }

        @Override
        public boolean update(double dtSeconds) {
            if (localizer.isConverged()) {
                locked = true;
                return true;
            }
            return timer.seconds() > timeoutSeconds;
        }

        @Override
        public void end() {
            drive.stop();
        }

        public boolean locked() {
            return locked;
        }
    }
}
