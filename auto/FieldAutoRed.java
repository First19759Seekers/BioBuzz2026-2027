package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Red alliance run of the field-localization autonomous.
 */
@Autonomous(name = "Auto: Field Tour (Red)", group = "Auto")
public class FieldAutoRed extends FieldAutoBase {

    @Override
    protected boolean isRedAlliance() {
        return true;
    }
}
