package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.localization.FieldPose;

/**
 * Waypoints of the demo autonomous, expressed in the RED alliance frame.
 * Blue is generated automatically by mirroring, so this file is edited once.
 *
 * All poses are (x inches, y inches, heading degrees) in the field frame
 * described in FieldConstants: origin at a field corner, +X and +Y along the
 * field edges, heading 0 = +X, positive = CCW.
 *
 * >>> THESE ARE EXAMPLE POSITIONS, NOT YOUR SEASON'S POSITIONS. <<<
 * The route deliberately crosses the whole field so that the localization is
 * exercised, and it is meant to be replaced with real scoring / pickup points.
 * Change the six numbers below, then the auto program drives the new route.
 */
public class AutoPoses {

    /** Where the robot sits at the start of auto (the first tag fix overrides it). */
    public static final FieldPose START = FieldPose.ofDegrees(12.0, 12.0, 0.0);

    /** First scoring spot, close to the alliance goal. */
    public static final FieldPose SCORE_PRELOAD = FieldPose.ofDegrees(36.0, 24.0, 0.0);

    /** First pickup, at the far end of the field: a long traverse on purpose. */
    public static final FieldPose PICKUP_ONE = FieldPose.ofDegrees(108.0, 24.0, 0.0);

    /** Back to score, on a different approach heading. */
    public static final FieldPose SCORE_CYCLE = FieldPose.ofDegrees(36.0, 72.0, 90.0);

    /** Second pickup across the diagonal. */
    public static final FieldPose PICKUP_TWO = FieldPose.ofDegrees(108.0, 108.0, 0.0);

    /** Final score. */
    public static final FieldPose SCORE_FINAL = FieldPose.ofDegrees(36.0, 120.0, 180.0);

    /** Park, clear of the scoring zone. */
    public static final FieldPose PARK = FieldPose.ofDegrees(12.0, 132.0, 180.0);

    private AutoPoses() {
    }
}
