package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name="Sandbox test", group="Test")
public class SandboxOpmode extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        while(opModeIsActive()){
            EncoderDrive(.5, 24, Reverse, stayOnHeading, 15);
            sleep(5000);
            EncoderDrive(.5, 24, Forward, stayOnHeading, 15);
            sleep(5000);
        }
    }
}