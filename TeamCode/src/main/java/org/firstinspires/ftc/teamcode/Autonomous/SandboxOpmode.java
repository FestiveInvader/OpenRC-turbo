package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name="Sandbox test", group="Test")
public class SandboxOpmode extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        while(opModeIsActive()){
            smartIntake();
            telemetry.addData("Left Vel", ConveyorLeft.getCurrentDraw());
            telemetry.addData("Right Vel", ConveyorRight.getCurrentDraw());
            telemetry.update();
        }
    }
}