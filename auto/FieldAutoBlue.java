package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Blue alliance run: the exact same mission, mirrored across the field.
 */
@Autonomous(name = "Auto: Field Tour (Blue)", group = "Auto")
public class FieldAutoBlue extends FieldAutoBase {

    @Override
    protected boolean isRedAlliance() {
        return false;
    }
}
