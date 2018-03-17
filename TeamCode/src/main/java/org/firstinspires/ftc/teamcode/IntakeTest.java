/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

*/
/**
 * Created by Eric Adams on 9/12/2017.
 *//*

@Disabled
@Autonomous
public class IntakeTest extends DeclarationsAutonomous {

    @Override
    public void runOpMode() {

        super.runOpMode();
        IntakeServoLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        IntakeServoRight.setDirection(DcMotorSimple.Direction.FORWARD);
        waitForStart();

        while (opModeIsActive()) {//Begin Opmode
            telemetry.addData("NotInLoop", "Before Read");
            telemetry.update();

            double distance = IntakeDistance.getDistance(DistanceUnit.CM);

            telemetry.addData("afterSensorRead", "Read");
            telemetry.update();

            while(true){
                telemetry.addData("InTrueLoop", IntakeDistance.getDistance(DistanceUnit.CM));
                telemetry.update();
                if(IntakeDistance.getDistance(DistanceUnit.CM) < 25){
                    IntakeServoLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                    IntakeServoRight.setDirection(DcMotorSimple.Direction.FORWARD);
                    IntakeServoLeft.setPower(1);
                    IntakeServoRight.setPower(1);
                    telemetry.addData("Distance", IntakeDistance.getDistance(DistanceUnit.CM));
                    telemetry.addData("ServoPowerLeft", IntakeServoLeft.getPower());
                    telemetry.addData("ServoPowerRight", IntakeServoRight.getPower());
                    telemetry.update();
                }else {
                    IntakeServoLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                    IntakeServoRight.setDirection(DcMotorSimple.Direction.FORWARD);
                    IntakeServoLeft.setPower(1);
                    IntakeServoRight.setPower(1);
                    telemetry.addData("ServoPowerLeft", IntakeServoLeft.getPower());
                    telemetry.addData("ServoPowerRight", IntakeServoRight.getPower());
                    telemetry.addData("Distance", IntakeDistance.getDistance(DistanceUnit.CM));
                    telemetry.update();
                }
            }
        }
    }
}
*/
