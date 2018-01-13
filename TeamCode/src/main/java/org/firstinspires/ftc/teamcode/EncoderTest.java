/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

//@Author Eric Adams, Team 8417 'Lectric Legends

@Autonomous(name="ENCODER TEST", group="TESTS")
@Disabled
// Was a concept opMode, as originally red auto got the 2nd beacon 1st, then got the 1st beacon 2nd(hah)
//This increase reliability, as problems that show up in one auto program most likely show up in the other.
public class EncoderTest extends Declerations {


    @Override
    public void runOpMode() {
        super.runOpMode();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && GyroSensor.isCalibrating()) {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", GyroSensor.rawZ());
            telemetry.update();
            idle();
        }
        GyroSensor.resetZAxisIntegrator();
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {//Begin Opmode
            while (opModeIsActive()) {
               // EncoderDrive(1, 100, 100, 10, 10, Forward);
                EncoderDrive(.25, 100, 100, 3, 5, Forward);
            }


        }// End of while Opmode is active
    }
    public void EncoderDrive(double speed, double leftInches, double rightInches, double accelerationInches, double decelerationInches, int direction) {
        // Declares variables that are used for this method
        int NewLeftTarget;
        int NewRightTarget;
        int RightPosition;
        int LeftPosition;

        double DecelTicks;
        double IntakeSpeed = speed;
        double LeftSpeed;
        double RightSpeed;
        double DistanceBeforeDeceleration;
        double DecelTickMultiplier;
        double DecelSpeedVar;
        double MinSpeed = .15;
        double SpeedToDecelerate;

        boolean Running = true;


        if (Running) {
            // Determine new target position, and pass to motor controller
            // Calculates the needed encoder ticks by multiplying a pre-determined amount of CountsPerInches,
            // and the method input gets the actual distance travel in inches
            NewLeftTarget = DriveRight.getCurrentPosition() + (int) (leftInches * CountsPerInch);
            NewRightTarget = DriveRight.getCurrentPosition() + (int) (rightInches * CountsPerInch);
            // Gets the current position of the encoders at the beginning of the EncoderDrive method
            RightPosition = DriveRight.getCurrentPosition();
            LeftPosition = DriveLeft.getCurrentPosition();

            // Setup for deceleration
            DecelTicks = ((decelerationInches * CountsPerInch));
            SpeedToDecelerate = speed - MinSpeed;
            DistanceBeforeDeceleration = Math.abs(NewLeftTarget) - Math.abs(DecelTicks);
            DecelTickMultiplier = (SpeedToDecelerate/DecelTicks);

            // Gives the encoders the target.
            DriveLeft.setTargetPosition(NewLeftTarget);
            DriveRight.setTargetPosition(NewRightTarget);

            // Resets encoders to 0
            DriveLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            DriveRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // Checks to make sure that encoders are reset.
            while(DriveLeft.getCurrentPosition() > 1 && DriveRight.getCurrentPosition()> 1){
                sleep(15);
            }
            // Turn On RUN_TO_POSITION
            DriveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            DriveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // reset the timeout time and start motion.
            runtime.reset();
            // This gets where the motor encoders will be at full position when it will be at full speed.
            double LeftEncoderPositionAtFullSpeed = ((accelerationInches*(CountsPerInch)) + LeftPosition);
            double RightEncoderPositionAtFullSpeed = ((accelerationInches*(CountsPerInch)) + RightPosition);

            // This gets the absolute value of the encoder positions at full speed - the current speed, and while it's greater than 0, it will continues increasing the speed.
            // This allows the robot to accelerate over a set number of inches, which reduces wheel slippage and increases overall reliability
            while (opModeIsActive() && Running) {// && opModeIsActive
                // While encoders are not at position
                if ((((Math.abs(speed)) - (Math.abs(DriveLeft.getPower()))) > .05) && (Math.abs(NewLeftTarget) - Math.abs(DriveLeft.getCurrentPosition()) > -1)
                        && (Math.abs(LeftEncoderPositionAtFullSpeed)- Math.abs(DriveLeft.getCurrentPosition()) > 10)){
                    LeftSpeed = (Range.clip(Math.abs((Math.abs(DriveLeft.getCurrentPosition())) / (Math.abs(LeftEncoderPositionAtFullSpeed))), MinSpeed, speed));
                    RightSpeed = (Range.clip(Math.abs((Math.abs(DriveRight.getCurrentPosition())) / (Math.abs(RightEncoderPositionAtFullSpeed))), MinSpeed, speed));
                    telemetry.addData("Accel Left IntakeSpeed", LeftSpeed);
                    telemetry.addData("Accel Right IntakeSpeed", RightSpeed);                    telemetry.addData("Target Accel", LeftEncoderPositionAtFullSpeed);
                    telemetry.addData("Position", DriveLeft.getCurrentPosition());
                    telemetry.addData("Distance/Inches Left", DriveLeft.getCurrentPosition()/CountsPerInch);
                    telemetry.addData("Distance/Inches Right", DriveRight.getCurrentPosition()/CountsPerInch);
                    telemetry.update();
                    //accelerating
                    // This allows the robot to accelerate over a set distance, rather than going full speed.  This reduces wheel slippage and increases reliability.
                }else if(((Math.abs(NewRightTarget)-Math.abs(DriveRight.getCurrentPosition()) <= Math.abs(DecelTicks)))
                        && (Math.abs(NewLeftTarget) - Math.abs(DriveLeft.getCurrentPosition()) > -1)){
                     // Ramp down the power
                    DecelSpeedVar = ((Math.abs(DriveLeft.getCurrentPosition())-Math.abs(DistanceBeforeDeceleration)));
                    double DecelClipVar = DecelSpeedVar* Math.abs(DecelTickMultiplier);
                    LeftSpeed = Range.clip(((Math.abs(speed) - Math.abs(DecelClipVar))), MinSpeed, speed);
                    RightSpeed = Range.clip(((Math.abs(speed) - Math.abs(DecelClipVar))), MinSpeed, speed);
                    telemetry.addData("Decel Left IntakeSpeed", LeftSpeed);
                    telemetry.addData("Decel Right IntakeSpeed", RightSpeed);                    telemetry.addData("Position", DriveLeft.getCurrentPosition());
                    telemetry.addData("Distance/Inches Left", DriveLeft.getCurrentPosition()/CountsPerInch);
                    telemetry.addData("Distance/Inches Right", DriveRight.getCurrentPosition()/CountsPerInch);
                    telemetry.update();
                }else{
                    RightSpeed = speed;
                    LeftSpeed = speed;
                    telemetry.addData("Normal Left IntakeSpeed", LeftSpeed);
                    telemetry.addData("Normal Right IntakeSpeed", RightSpeed);
                    telemetry.addData("Distance/Inches Left", DriveLeft.getCurrentPosition()/CountsPerInch);
                    telemetry.addData("Distance/Inches Right", DriveRight.getCurrentPosition()/CountsPerInch);
                    telemetry.update();
                }
                if(Math.abs(NewLeftTarget) - Math.abs(DriveLeft.getCurrentPosition()) < -1) {
                    //If absolute value of wanted encoder count at finish -
                    // the absolute value of current motor is < -1, then stop running.
                    Running = false;
                }
                DriveLeft.setPower(Range.clip(LeftSpeed, MinSpeed*direction,direction));
                DriveRight.setPower(Range.clip(RightSpeed, MinSpeed*direction,direction));
            }

            // Stops all motion
            // Set to run without encoder, so it's not necessary to declare this every time after the method is used
            DriveLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            DriveRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // Set power to 0
            DriveLeft.setPower(0);
            DriveRight.setPower(0);

        }
    }


}*/
