package org.firstinspires.ftc.teamcode.localization;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Iterator;

/**
 * The full-field localization system.
 *
 * Two independent estimates are combined every loop:
 *
 *   prediction  - wheel encoders + gyro ({@link Odometry}). Smooth, always
 *                 available, drifts slowly.
 *   measurement - AprilTags through the Limelight ({@link VisionEstimator}).
 *                 Absolute, but only when a tag is in view, and it arrives
 *                 ~30-80 ms late.
 *
 * Three details make this behave like a real sensor fusion instead of a naive
 * "copy the camera pose into the robot":
 *
 *   1. Latency compensation. Every fused pose is timestamped into a short
 *      history ring. A vision frame is compared against the prediction *at the
 *      moment the image was captured*, not at the moment it arrived, so a
 *      moving robot does not fight its own camera.
 *   2. Residual gating. A fix that disagrees with the prediction by more than a
 *      sane robot motion (or whose tags disagree with each other) is rejected
 *      and counted, which is what keeps a single mis-identified tag from
 *      throwing the autonomous program across the field.
 *   3. Weighted correction. Each accepted fix only pulls the pose part of the
 *      way toward the measurement, so the output stays continuous and the
 *      drivetrain never receives a step input.
 *
 * Heading is owned by the gyro; tags only nudge it, because a wrong tag facing
 * convention shows up as a constant heading bias that must never be allowed to
 * spin the robot.
 */
public class FieldLocalizer {

    /**
     * The very first tag fix is allowed to be far away: the configured start
     * pose is only a guess and the whole point of the first fix is to replace it.
     */
    private static final double FIRST_FIX_MAX_RESIDUAL_IN = 96.0;

    private static class Sample {
        final long timeNanos;
        final FieldPose pose;

        Sample(long timeNanos, FieldPose pose) {
            this.timeNanos = timeNanos;
            this.pose = pose;
        }
    }

    private final Odometry odometry;
    private final LimelightVision vision;
    private final VisionEstimator estimator = new VisionEstimator();
    private final Deque<Sample> history = new ArrayDeque<>();

    private FieldPose pose = FieldPose.ZERO;
    private FieldPose lastVisionPose;

    private long lastVisionTimeNanos;
    private boolean everHadVision;

    private int visibleTags;
    private int acceptedFrames;
    private int rejectedFrames;
    private double residualIn;
    private double headingResidualDeg;
    private double tagSpreadIn;
    private String lastRejectReason = "waiting for tag";

    public FieldLocalizer(MecanumDrive drive, HardwareMap hardwareMap) {
        this.odometry = new Odometry(drive);
        this.vision = new LimelightVision(hardwareMap);
    }

    /** Call exactly once per robot loop, before reading {@link #getPose()}. */
    public void update() {
        long now = System.nanoTime();

        // Give MegaTag2 the yaw it cannot recover from the image alone.
        vision.setRobotOrientation(pose.heading);

        FieldPose previousOdom = odometry.getPose();
        odometry.update();
        FieldPose odom = odometry.getPose();

        double dx = odom.x - previousOdom.x;
        double dy = odom.y - previousOdom.y;
        double dHeading = FieldPose.normalizeAngle(odom.heading - previousOdom.heading);
        pose = pose.plus(dx, dy, dHeading);

        record(now, pose);

        LimelightVision.Frame frame = vision.poll();
        if (frame != null) {
            fuse(frame, now);
        }
    }

    private void fuse(LimelightVision.Frame frame, long now) {
        long captureTime = frame.timeNanos;
        if (captureTime <= 0 || Math.abs(now - captureTime) > 2_000_000_000L) {
            // Timestamp looks unrelated to our clock: fall back to arrival time
            // minus the reported pipeline latency.
            captureTime = now - (long) (frame.latencyMs * 1.0e6);
        }

        FieldPose predicted = poseAt(captureTime);

        VisionEstimator.Estimate estimate = estimator.estimate(frame, predicted.heading);
        visibleTags = frame.tags.size();

        if (!estimate.valid) {
            rejectedFrames++;
            lastRejectReason = estimate.note;
            return;
        }

        double positionResidual = Math.hypot(estimate.x - predicted.x, estimate.y - predicted.y);
        double headingResidual = FieldPose.normalizeAngle(estimate.heading - predicted.heading);

        residualIn = positionResidual;
        headingResidualDeg = Math.toDegrees(headingResidual);
        tagSpreadIn = estimate.spreadIn;
        visibleTags = estimate.tagCount;

        if (estimate.spreadIn > VisionConstants.MAX_TAG_SPREAD_IN) {
            rejectedFrames++;
            lastRejectReason = String.format("tag disagreement %.1fin", estimate.spreadIn);
            return;
        }

        boolean firstFix = acceptedFrames == 0;
        if (!firstFix && positionResidual > VisionConstants.MAX_TRANSLATION_RESIDUAL_IN) {
            rejectedFrames++;
            lastRejectReason = String.format("position jump %.1fin", positionResidual);
            return;
        }
        if (firstFix && positionResidual > FIRST_FIX_MAX_RESIDUAL_IN) {
            rejectedFrames++;
            lastRejectReason = String.format("first fix too far %.1fin", positionResidual);
            return;
        }

        double baseGain = estimate.tagCount >= 2
                ? VisionConstants.TRANSLATION_GAIN_MULTI_TAG
                : VisionConstants.TRANSLATION_GAIN_ONE_TAG;

        // Distant tags are noisier: fade the gain out instead of trusting them fully.
        double distanceFactor = clamp(1.0 - (estimate.nearestTagIn - 24.0) / 120.0, 0.4, 1.0);
        double gain = firstFix ? 1.0 : clamp(baseGain * distanceFactor, 0.0, 1.0);

        double correctionX = (estimate.x - predicted.x) * gain;
        double correctionY = (estimate.y - predicted.y) * gain;
        double correctionHeading = 0.0;

        if (VisionConstants.USE_TAG_HEADING
                && VisionConstants.HEADING_GAIN > 0.0
                && Math.abs(headingResidualDeg) < VisionConstants.HEADING_FIX_MAX_ERROR_DEG) {
            correctionHeading = clamp(
                    VisionConstants.HEADING_GAIN * headingResidual,
                    -Math.toRadians(VisionConstants.MAX_HEADING_CORRECTION_DEG),
                    Math.toRadians(VisionConstants.MAX_HEADING_CORRECTION_DEG));
        }

        pose = pose.plus(correctionX, correctionY, correctionHeading);

        lastVisionPose = estimate.toPose();
        lastVisionTimeNanos = now;
        everHadVision = true;
        acceptedFrames++;
        lastRejectReason = "ok";
    }

    /** Fused pose, in the field frame. This is what everything else consumes. */
    public FieldPose getPose() {
        return pose;
    }

    public FieldPose getOdometryPose() {
        return odometry.getPose();
    }

    public FieldPose getLastVisionPose() {
        return lastVisionPose;
    }

    public LimelightVision getVision() {
        return vision;
    }

    public boolean hasVision() {
        return everHadVision && millisSinceVision() < VisionConstants.VISION_TIMEOUT_MS;
    }

    public double millisSinceVision() {
        if (!everHadVision) {
            return Double.MAX_VALUE;
        }
        return (System.nanoTime() - lastVisionTimeNanos) / 1.0e6;
    }

    public int getVisibleTags() {
        return visibleTags;
    }

    public int getAcceptedFrames() {
        return acceptedFrames;
    }

    public int getRejectedFrames() {
        return rejectedFrames;
    }

    public double getResidualIn() {
        return residualIn;
    }

    public double getHeadingResidualDeg() {
        return headingResidualDeg;
    }

    public double getTagSpreadIn() {
        return tagSpreadIn;
    }

    public String getLastRejectReason() {
        return lastRejectReason;
    }

    /** True when a recent, self-consistent tag fix is available. */
    public boolean isConverged() {
        return hasVision() && residualIn <= VisionConstants.CONVERGED_RESIDUAL_IN;
    }

    public void setPose(FieldPose newPose) {
        pose = newPose;
        odometry.setPose(newPose);
        history.clear();
        lastVisionPose = null;
        everHadVision = false;
        residualIn = 0.0;
    }

    /**
     * Re-zeroes the dead-reckoning reference onto the fused pose without losing
     * the vision history. Called at the start of auto so encoder drift from the
     * init period is not carried into the run.
     */
    public void anchor() {
        odometry.setPose(pose);
    }

    public String getStatusLine() {
        return String.format("vis %s tags %d res %.1fin spread %.1f rej %d %s",
                hasVision() ? String.format("%.0fms", millisSinceVision()) : "none",
                visibleTags,
                residualIn,
                tagSpreadIn,
                rejectedFrames,
                lastRejectReason);
    }

    // ------------------------------------------------------------------
    // Pose history, used to answer "where was I when that picture was taken?"
    // ------------------------------------------------------------------

    private void record(long timeNanos, FieldPose sample) {
        history.addLast(new Sample(timeNanos, sample));
        long cutoff = timeNanos - (long) (VisionConstants.HISTORY_LENGTH_MS * 1.0e6);
        while (!history.isEmpty() && history.peekFirst().timeNanos < cutoff) {
            history.removeFirst();
        }
    }

    private FieldPose poseAt(long timeNanos) {
        if (history.isEmpty()) {
            return pose;
        }

        Sample previous = null;
        Iterator<Sample> iterator = history.iterator();
        while (iterator.hasNext()) {
            Sample sample = iterator.next();
            if (sample.timeNanos >= timeNanos) {
                if (previous == null) {
                    return sample.pose;
                }
                long span = sample.timeNanos - previous.timeNanos;
                if (span <= 0) {
                    return sample.pose;
                }
                double ratio = (double) (timeNanos - previous.timeNanos) / (double) span;
                return FieldPose.interpolate(previous.pose, sample.pose, clamp(ratio, 0.0, 1.0));
            }
            previous = sample;
        }

        // Older than the newest sample: extrapolate using the last known velocity.
        Sample last = history.peekLast();
        Sample secondLast = null;
        for (Sample sample : history) {
            if (sample == last) {
                break;
            }
            secondLast = sample;
        }
        if (last == null || secondLast == null) {
            return pose;
        }

        long span = last.timeNanos - secondLast.timeNanos;
        if (span <= 0) {
            return last.pose;
        }

        double dtSeconds = Math.min((timeNanos - last.timeNanos) / 1.0e9,
                VisionConstants.MAX_EXTRAPOLATION_MS / 1000.0);
        double vx = (last.pose.x - secondLast.pose.x) / (span / 1.0e9);
        double vy = (last.pose.y - secondLast.pose.y) / (span / 1.0e9);
        double omega = FieldPose.normalizeAngle(last.pose.heading - secondLast.pose.heading)
                / (span / 1.0e9);

        return last.pose.plus(vx * dtSeconds, vy * dtSeconds, omega * dtSeconds);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
