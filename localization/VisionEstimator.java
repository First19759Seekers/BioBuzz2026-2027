package org.firstinspires.ftc.teamcode.localization;

import java.util.List;

/**
 * Turns one camera frame into a field pose.
 *
 * This is a MegaTag-style solve done in 2D against {@link AprilTagMap}:
 * each visible tag gives the robot's position by subtracting the tag's measured
 * offset from the tag's known field position, and the results of all visible
 * tags are averaged with a distance/area weighting. Using the IMU heading as the
 * rotation for that subtraction makes a single-tag fix usable, which is the same
 * trick MegaTag2 relies on.
 *
 * Heading is reported as an independent observation: the localizer decides
 * whether to trust it, because a misconfigured tag facing convention shows up
 * here as a constant heading error while the position solve stays perfectly fine.
 */
public class VisionEstimator {

    public static class Estimate {
        public final boolean valid;
        public final double x;
        public final double y;
        public final double heading;
        public final int tagCount;
        public final double spreadIn;
        public final double nearestTagIn;
        public final String note;

        private Estimate(boolean valid, double x, double y, double heading,
                         int tagCount, double spreadIn, double nearestTagIn, String note) {
            this.valid = valid;
            this.x = x;
            this.y = y;
            this.heading = heading;
            this.tagCount = tagCount;
            this.spreadIn = spreadIn;
            this.nearestTagIn = nearestTagIn;
            this.note = note;
        }

        static Estimate invalid(String note) {
            return new Estimate(false, 0, 0, 0, 0, 0, 0, note);
        }

        public FieldPose toPose() {
            return new FieldPose(x, y, heading);
        }
    }

    public Estimate estimate(LimelightVision.Frame frame, double headingGuessRad) {
        List<LimelightVision.TagObservation> tags = frame.tags;
        if (tags.isEmpty()) {
            return Estimate.invalid("no usable tags");
        }

        double cos = Math.cos(headingGuessRad);
        double sin = Math.sin(headingGuessRad);

        double weightSum = 0.0;
        double xSum = 0.0;
        double ySum = 0.0;
        double headingSinSum = 0.0;
        double headingCosSum = 0.0;
        double headingWeightSum = 0.0;
        double nearest = Double.MAX_VALUE;
        int used = 0;

        double[] xs = new double[tags.size()];
        double[] ys = new double[tags.size()];

        for (LimelightVision.TagObservation observation : tags) {
            AprilTagMap.Tag tag = AprilTagMap.get(observation.id);
            if (tag == null) {
                continue;
            }

            // Camera mount offsets are added here so every observation refers to
            // the robot's rotation center. Leave them at 0 when the mount is
            // already configured inside the Limelight web UI.
            double forward = observation.forwardIn + VisionConstants.CAMERA_FORWARD_OFFSET_IN;
            double left = observation.leftIn + VisionConstants.CAMERA_LEFT_OFFSET_IN;

            double tagX = tag.x - (cos * forward - sin * left);
            double tagY = tag.y - (sin * forward + cos * left);

            double distance = observation.distanceIn;
            double weight = (0.2 + Math.min(observation.area, 0.2))
                    / (1.0 + (distance / 48.0) * (distance / 48.0));

            xSum += tagX * weight;
            ySum += tagY * weight;
            weightSum += weight;

            // Phi is the tag facing direction in the field, psi the same
            // direction seen from the robot: theta = phi - psi.
            double headingObservation = FieldPose.normalizeAngle(tag.yaw - observation.yawInRobotRad);
            headingSinSum += Math.sin(headingObservation) * weight;
            headingCosSum += Math.cos(headingObservation) * weight;
            headingWeightSum += weight;

            if (distance < nearest) {
                nearest = distance;
            }

            xs[used] = tagX;
            ys[used] = tagY;
            used++;
        }

        if (used == 0 || weightSum <= 0.0) {
            return Estimate.invalid("tags not in field map");
        }

        double meanX = xSum / weightSum;
        double meanY = ySum / weightSum;

        // Spread is the worst disagreement between individual tags. A large
        // spread means a wrong tag position in the map, a wrong ID or a
        // reflection/mirroring issue, never a real robot motion.
        double spread = 0.0;
        for (int i = 0; i < used; i++) {
            double deviation = Math.hypot(xs[i] - meanX, ys[i] - meanY);
            if (deviation > spread) {
                spread = deviation;
            }
        }

        double heading = Math.atan2(headingSinSum, headingCosSum);

        return new Estimate(true, meanX, meanY, heading, used, spread, nearest, "ok");
    }
}
