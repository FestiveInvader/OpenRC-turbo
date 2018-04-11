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
        double startingRotation = getHeading();
        boolean foundGlyph = false;
        while(opModeIsActive() && !foundGlyph){
            if(IntakeDistance.getDistance(DistanceUnit.CM) > 10){
                gyroTurn(turningSpeed, -startingRotation - 25);
                foundGlyph = true;
            }else {
                moveBy(.2, 0, 0);
            }
            smartIntake();

        }
    }
}