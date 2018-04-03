package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Sandbox test", group="Test")
public class SandboxOpmode extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        while(opModeIsActive()){
            goToDistance(.25, 50, BackDistance, 5, 4);
            sleep(10000);
        }
    }
}