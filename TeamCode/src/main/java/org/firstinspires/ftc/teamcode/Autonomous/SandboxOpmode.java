package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name="Sandbox test", group="Test")
public class SandboxOpmode extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        while(opModeIsActive()){
            driveWStrafe(-.15,.3,0,1);
            sleep(2500);
            driveWStrafe(-.15,-.6,0,1);
            sleep(2500);
            /*driveWStrafe(0,.3,0,1);
            sleep(2500);
            driveWStrafe(0, -.6, 0, 1);
            sleep(2500);*/
        }
    }
}