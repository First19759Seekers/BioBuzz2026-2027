package org.firstinspires.ftc.teamcode.localization;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Thin, non-blocking wrapper around the Limelight 3A.
 *
 * The camera is polled from the main robot loop, never from its own thread, and
 * every frame is converted into plain inches/radians so the localization layer
 * does not need to know anything about the vendor API.
 *
 * This is the direct descendant of FatDragon's AutoApriltag subsystem: the
 * camera ownership, poll rate and start/stop lifecycle are the same idea,
 * extended with per-tag geometry, timestamps and latency so the pose can be
 * fused instead of merely read.
 */
public class LimelightVision {

    /** One visible tag, expressed in the robot frame (inches, radians). */
    public static class TagObservation {
        public final int id;
        /** Distance ahead of the camera. */
        public final double forwardIn;
        /** Distance to the robot's left. */
        public final double leftIn;
        /** Yaw of the tag face relative to the robot. */
        public final double yawInRobotRad;
        /** Fraction of the image covered by the tag. */
        public final double area;
        public final double distanceIn;

        TagObservation(int id, double forwardIn, double leftIn, double yawInRobotRad, double area) {
            this.id = id;
            this.forwardIn = forwardIn;
            this.leftIn = leftIn;
            this.yawInRobotRad = yawInRobotRad;
            this.area = area;
            this.distanceIn = Math.hypot(forwardIn, leftIn);
        }
    }

    /** Immutable snapshot of one camera frame. */
    public static class Frame {
        public final long timeNanos;
        public final double latencyMs;
        public final List<TagObservation> tags;

        /** True when the Limelight itself produced a valid MegaTag2 pose. */
        public final boolean mt2Valid;
        public final double mt2XIn;
        public final double mt2YIn;
        public final double mt2HeadingRad;
        public final int mt2TagCount;

        Frame(long timeNanos, double latencyMs, List<TagObservation> tags,
              boolean mt2Valid, double mt2XIn, double mt2YIn, double mt2HeadingRad, int mt2TagCount) {
            this.timeNanos = timeNanos;
            this.latencyMs = latencyMs;
            this.tags = Collections.unmodifiableList(tags);
            this.mt2Valid = mt2Valid;
            this.mt2XIn = mt2XIn;
            this.mt2YIn = mt2YIn;
            this.mt2HeadingRad = mt2HeadingRad;
            this.mt2TagCount = mt2TagCount;
        }
    }

    private Limelight3A limelight;
    private boolean available;
    private long lastFrameStamp = Long.MIN_VALUE;

    public LimelightVision(HardwareMap hardwareMap) {
        // Accept the common naming variants so a config named "Limelight" or
        // "ll" does not silently degrade the whole localization to odometry.
        String[] candidates = {VisionConstants.LIMELIGHT_NAME, "Limelight", "limelight3a", "ll"};
        for (String name : candidates) {
            try {
                limelight = hardwareMap.get(Limelight3A.class, name);
                break;
            } catch (RuntimeException ignored) {
                limelight = null;
            }
        }

        if (limelight == null) {
            available = false;
            return;
        }

        try {
            limelight.setPollRateHz(VisionConstants.POLL_RATE_HZ);
            if (VisionConstants.SWITCH_PIPELINE_ON_START) {
                limelight.pipelineSwitch(VisionConstants.PIPELINE_INDEX);
            }
            limelight.start();
            available = true;
        } catch (RuntimeException e) {
            available = false;
        }
    }

    public boolean isAvailable() {
        return available;
    }

    public String getStatusLine() {
        if (!available) {
            return "Limelight not configured";
        }
        try {
            return String.format("LL %.0fC %.0ffps p%d",
                    limelight.getStatus().getTemp(),
                    limelight.getStatus().getFps(),
                    limelight.getStatus().getPipelineIndex());
        } catch (RuntimeException e) {
            return "Limelight offline";
        }
    }

    /**
     * Tells the camera which way the robot is facing, in degrees, field frame.
     * MegaTag2 needs this: with a single tag the yaw cannot be recovered from
     * the image alone, so feeding the fused heading makes the fix far steadier.
     */
    public void setRobotOrientation(double fieldHeadingRad) {
        if (!available) {
            return;
        }
        try {
            limelight.updateRobotOrientation(Math.toDegrees(FieldPose.normalizeAngle(fieldHeadingRad)));
        } catch (RuntimeException e) {
            // Camera hiccup: never let vision kill the match.
        }
    }

    /**
     * Returns a new frame, or null when there is nothing new since the last call.
     * Called once per robot loop, so it must stay cheap and must never block.
     */
    public Frame poll() {
        if (!available) {
            return null;
        }

        LLResult result;
        try {
            result = limelight.getLatestResult();
        } catch (RuntimeException e) {
            return null;
        }
        if (result == null || !result.isValid()) {
            return null;
        }

        long stamp = result.getControlHubTimeStampNanos();
        // Limelight reports these three stages in milliseconds.
        double latencyMs = result.getCaptureLatency() + result.getTargetingLatency() + result.getParseLatency();
        if (stamp <= 0) {
            stamp = System.nanoTime() - (long) (latencyMs * 1.0e6);
        }
        if (stamp == lastFrameStamp) {
            return null;
        }
        lastFrameStamp = stamp;

        List<TagObservation> tags = new ArrayList<>();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials != null) {
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                TagObservation observation = toObservation(fiducial);
                if (observation != null) {
                    tags.add(observation);
                }
            }
        }

        boolean mt2Valid = false;
        double mt2X = 0.0;
        double mt2Y = 0.0;
        double mt2Heading = 0.0;
        Pose3D botpose = result.getBotpose_MT2();
        if (botpose != null) {
            Position position = botpose.getPosition().toUnit(DistanceUnit.INCH);
            mt2X = position.x;
            mt2Y = position.y;
            mt2Heading = botpose.getOrientation().getYaw(AngleUnit.RADIANS);
            mt2Valid = true;
        }

        return new Frame(stamp, latencyMs, tags, mt2Valid, mt2X, mt2Y, mt2Heading,
                result.getBotposeTagCount());
    }

    private TagObservation toObservation(LLResultTypes.FiducialResult fiducial) {
        int id = fiducial.getFiducialId();
        if (!AprilTagMap.contains(id)) {
            return null;
        }

        double area = fiducial.getTargetArea();
        if (area < VisionConstants.MIN_TAG_AREA_FRACTION) {
            return null;
        }

        Pose3D targetPose = fiducial.getTargetPoseRobotSpace();
        if (targetPose == null) {
            return null;
        }

        Position position = targetPose.getPosition().toUnit(DistanceUnit.INCH);
        double forward = position.x;
        double left = position.y * VisionConstants.ROBOT_SPACE_LEFT_SIGN;
        double distance = Math.hypot(forward, left);
        if (distance > VisionConstants.MAX_TAG_DISTANCE_IN || distance < 1.0) {
            return null;
        }

        double yaw = targetPose.getOrientation().getYaw(AngleUnit.RADIANS)
                + Math.toRadians(VisionConstants.TAG_YAW_OFFSET_DEG);

        return new TagObservation(id, forward, left, FieldPose.normalizeAngle(yaw), area);
    }

    public void stop() {
        if (!available) {
            return;
        }
        try {
            limelight.stop();
        } catch (RuntimeException e) {
            // Ignore: the OpMode is already tearing down.
        }
    }
}
