package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.TimeUnit;

@Autonomous(name="Sandbox test", group="Test")
public class SandboxOpmode extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        boolean linedUp = false;
        while(!linedUp && opModeIsActive() && runtime.seconds() < 25)  {

                /*if(glyphs = 0){
                    //we want a brown glyph
                }else{
                    //we want a grey glyph
                }*/


            if(FrontDistance.getDistance() <= 22 && FrontDistance.getDistance() > 20 ){
                linedUp = true;
            }else{
                if(FrontRightDistance.getDistance(DistanceUnit.CM) > 1 &&
                        FrontLeftDistance.getDistance(DistanceUnit.CM) > 1){
                    //if starting position, make sure we turn relic side
                    moveBy(.175, 0, 0);
                }else if (FrontLeftDistance.getDistance(DistanceUnit.CM) > 1) {
                    moveBy(.1,0,.35);
                } else if (FrontRightDistance.getDistance(DistanceUnit.CM) > 1) {
                    moveBy(.1,0,-.35);
                } else {
                    moveBy(.15, 0, 0);
                }
            }
            smartIntake();
            telemetry.addData("Lining Up", 0);
            telemetry.addData("Frnt Left D", FrontLeftDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Frnt Right D", FrontRightDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Frnt  D", FrontDistance.getDistance());
            telemetry.update();

        }
        intakeGlyph();
        stopDriveMotors();
        telemetry.addData("Lining Up", 0);
        telemetry.addData("Frnt Left D", FrontLeftDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Frnt Right D", FrontRightDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Frnt  D", FrontDistance.getDistance());
        telemetry.addData("lined up", linedUp);
        telemetry.update();
        sleep(10000);
    }

}