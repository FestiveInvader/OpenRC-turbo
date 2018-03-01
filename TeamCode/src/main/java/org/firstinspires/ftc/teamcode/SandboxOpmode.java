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




    }
    public void MGPickup(){
        driveToGlyphs();
    }
    public void driveToGlyphs(){
        int glyphs = 0;
        double timer;
        boolean wentLeft = false;
        boolean wentRight = false;

        while(runtime.seconds() < 25 && glyphs < 2 && opModeIsActive()){
            boolean linedUp = false;


            while(!linedUp && opModeIsActive() && runtime.seconds() < 25 && !haveGlyph())  {
                /*if(glyphs = 0){
                    //we want a brown glyph
                }else{
                    //we want a grey glyph
                }*/
                if (FrontLeftDistance.getDistance(DistanceUnit.CM) > 1) {
                    timer = runtime.seconds();
                    while (FrontLeftDistance.getDistance(DistanceUnit.CM) > 1 && opModeIsActive()) {
                        moveBy(0, -.425, 0);
                    }
                    wentLeft = true;
                    linedUp = true;
                } else if (FrontRightDistance.getDistance(DistanceUnit.CM) > 1) {
                    timer = runtime.seconds();
                    while (FrontLeftDistance.getDistance(DistanceUnit.CM) > 1 && opModeIsActive()) {
                        moveBy(0, .425, 0);
                    }
                    wentRight = true;
                    linedUp = true;
                } else {
                    moveBy(.2, 0, 0);
                }

            }
            if(haveGlyph()){
                glyphs += 1;
            }
            intakeGlyph();

        }
    }
    public boolean haveGlyph(){
        boolean haveGlyph = false;
        double SensorVal = IntakeDistance.getDistance(DistanceUnit.CM);
        if (SensorVal <= 14) {
            haveGlyph = true;
        }
        return  haveGlyph;
    }
    public void smartIntake(){
        double SensorVal = IntakeDistance.getDistance(DistanceUnit.CM);
        if (SensorVal <= 14) {
            IntakeServoLeft.setPower(IntakeSpeed);
            IntakeServoRight.setPower(-IntakeSpeed);
        }else if(SensorVal > 14 && SensorVal < 20){
            IntakeServoLeft.setPower(IntakeSpeed);
            IntakeServoRight.setPower(IntakeSpeed);
        }else if (SensorVal >= 20){
            IntakeServoLeft.setPower(-IntakeSpeed);
            IntakeServoRight.setPower(-IntakeSpeed);
        }else{
            IntakeServoLeft.setPower(IntakeSpeed);
            IntakeServoRight.setPower(-IntakeSpeed);
        }
    }
    public void smartIntakeDelay(double delay){
        double timeLimit = runtime.time(TimeUnit.SECONDS);
        while(runtime.seconds() - timeLimit < delay){
            smartIntake();
        }
    }
    public void intakeGlyph() {
        boolean haveGlyph = false;
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double startingEncoderCount = FrontLeft.getCurrentPosition();
        double limitEncoderCount = startingEncoderCount + 6*CountsPerInch;
        DumpConveyor.setPower(1);
        ConveyorLeft.setPower(1);
        ConveyorRight.setPower(1);
        TopIntakeServoLeft.setPower(1);
        TopIntakeServoRight.setPower(1);
        while(!haveGlyph && (Math.abs(FrontLeft.getCurrentPosition()) < Math.abs(limitEncoderCount)) && runtime.seconds() < 24){
            if(IntakeDistance.getDistance(DistanceUnit.CM) < 14){
                stopDriveMotors();
                haveGlyph = true;
            }else {
                moveBy(.2, 0, 0);
                smartIntake();
            }
        }
        smartIntakeDelay(.5);
        telemetry.update();
        smartIntakeDelay(1);
        IntakeServoLeft.setPower(IntakeSpeed);
        IntakeServoRight.setPower(-IntakeSpeed);
        double inchesToDrive = FrontLeft.getCurrentPosition()/CountsPerInch;
        EncoderDrive(1, Math.abs(inchesToDrive), Reverse, stayOnHeading, 5);
        glyphs += 1;


        /*if(IntakeColor.alpha() > 130){
            telemetry.addData("Grey", IntakeColor.alpha());
            color = 1;
        }else{
            telemetry.addData("Brown", IntakeColor.alpha());
            color = 2;
        }*/
    }
}