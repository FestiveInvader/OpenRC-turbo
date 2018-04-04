package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name="Sandbox test", group="Test")
public class SandboxOpmode extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        while(opModeIsActive()){
            EncoderDriveAccelDecel(.5, 20, 8, Forward, stayOnHeading, 8);
            sleep(2000);
            EncoderDriveAccelDecel(.5, 20, 8, Reverse, stayOnHeading, 8);
            sleep(2000);

        }
    }
}