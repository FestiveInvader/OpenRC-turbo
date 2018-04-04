package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name="Sandbox test", group="Test")
public class SandboxOpmode extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        while(opModeIsActive()){
            EncoderDrive(.75, 25, 10, 10, Forward, stayOnHeading, 8);
            sleep(1000);
            EncoderDrive(.75, 25, 10, 10, Reverse,stayOnHeading, 8);
            sleep(1000);

        }
    }
}