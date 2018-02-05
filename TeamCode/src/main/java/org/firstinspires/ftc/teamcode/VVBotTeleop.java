

package org.firstinspires.ftc.teamcode;

//@Author Eric Adams, Co-Captain and Lead Programmer for Team 8417, The Lectric Legends

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Disabled
@TeleOp(name="VVTELEOP", group="TELEOP")

public class VVBotTeleop extends OpMode {

    // This section declares hardware for the program, such as DC Motors, servos and sensors
    DcMotorSimple Shooter1 = null; // Andymark NeveRest 3.7
    DcMotorSimple Shooter2 = null; // Andymark NeveRest 3.7
    DcMotorSimple ExternalBeaterBar; // Andymark NeveRest 20
    //DcMotorSimple BeaconLights = null; // 12V Green LEDs
    //DcMotorSimple LinearSlideMotor = null;
    DcMotor DriveRight = null; // Andymark NeveRest 40 with encoder
    DcMotor DriveLeft = null; // Andymark NeveRest 40 with encoder
    DcMotor LoaderMotor = null; // Tetrix 152 RPM DC motor

        double ShooterPower;
     double Shooter2Power;
    //double SpeedMultiplier = 1;
    float RightPower;
    float LeftPower;
    float ExternalBeaterBarPower;

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void init() {
        //  This section gets the hardware names and addresses from the config file stored in the
        //  Robot Controller and assigns the declared motors to the correct motor addresses
        DriveRight = hardwareMap.dcMotor.get("DriveRight");
        DriveLeft = hardwareMap.dcMotor.get("DriveLeft");
        Shooter1 = hardwareMap.dcMotor.get("Shooter1");
        Shooter2 = hardwareMap.dcMotor.get("Shooter2");
        ExternalBeaterBar = hardwareMap.dcMotor.get("ExternalBeaterBar");
        LoaderMotor = hardwareMap.dcMotor.get("Loader");
        // This section sets the default direction of motors.  This allows for ease of programming,
        // mostly for drive motors, but is useful for others as well.
        DriveRight.setDirection(DcMotor.Direction.FORWARD);
        DriveLeft.setDirection(DcMotor.Direction.REVERSE);
        ExternalBeaterBar.setDirection(DcMotorSimple.Direction.FORWARD);
        LoaderMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        // This section is declaring the servos used on the robot.

        // Put a line of text on the driver station app that says Status: Initialized
        telemetry.addData("Status of Program:", "Initialized");
    }

    @Override
    public void init_loop() {
        // On initialization.  This is where the servos are set to their starting positions.
        //SideBeaconPusher.setPosition(SideBeaconInPos);
    }

    @Override
    public void start() {
        //This resets the time to the start time, which may or may not have been a large discrepancy
        runtime.reset();
    }


    @Override
    public void loop() {

       if (gamepad1.right_bumper || gamepad2.right_bumper) {
            ExternalBeaterBarPower = 1;
        }else if (gamepad2.dpad_down) {
            ExternalBeaterBarPower = -1;
        }else{
            ExternalBeaterBarPower = 0;
        }
        // if the left trigger on controller 2 is held down more than halfway, the beater bar will rotate
        // so it pushes any particles it comes into contact with away.

        //When the Dpad Up button on gamepad 1 or gamepad 2 is pressed, the Loader motor is set to full speed
        if (gamepad1.dpad_up || gamepad2.dpad_up) {
            LoaderMotor.setPower(1);
        }else {
            //When dpad down on gamepad 1 or gamepad 2 isn't pressed, the Loader Motor is stopped
            LoaderMotor.setPower(0);
        }

        //  The loop Makes the robot go slower than its regular speed when
        //  the left trigger is pressed down more than 2% of the way.  If its pressed more than 80% of the way down,
        // then the robot will go much slower. If it's not pressed down,
        // the program sets the drive motor power variables to gamepad1 joysticks values, and scales them
        // to an array of variables declared below.
        if(gamepad1.right_trigger > .1) {
            LeftPower = gamepad1.left_stick_y / (float) 4;
            RightPower = gamepad1.right_stick_y / (float) 4;
            RightPower = Range.clip(RightPower, -1, 1);
            LeftPower = Range.clip(LeftPower, -1, 1);
        } else if(gamepad1.left_trigger > .1){
            LeftPower = gamepad1.left_stick_y / (float) 2;
            RightPower = gamepad1.right_stick_y / (float) 2;
            RightPower = Range.clip(RightPower, -1, 1);
            LeftPower = Range.clip(LeftPower, -1, 1);
        }else{
            LeftPower = (ScaleInputDrive(gamepad1.left_stick_y));
            RightPower = (ScaleInputDrive(gamepad1.right_stick_y));
        }
        // For shots that do need speed variance, this function sets the ShooterPower to the Right trigger on gamepad2,
        // Allowing for small variances in position of the robot by having the power set to either
        // 0 power, 75 power, or 100 power
//        Shooter2Power = (ScaleShooterRightTrigger(gamepad2.right_trigger + gamepad1.right_trigger)+(ScaleShooterLeftTrigger(gamepad2.left_trigger)));
        ShooterPower =  (ScaleShooterRightTrigger(gamepad2.right_trigger)+(ScaleShooterLeftTrigger(gamepad2.left_trigger)));
        ShooterPower = Range.clip(ShooterPower, 0, 1);

        //This is where the motors actually get the power set to them.  Before, I was just
        //Editing a placeholder, because I can't always use the .setPower with the input data
        //That is collected.

        //Sets the beater bar on the front of the robot the the ExternalBeaterBarPower variable
        ExternalBeaterBar.setPower(ExternalBeaterBarPower);
        //Sets Shooter motors(2 NeveRest motors with 3.7:1 gearboxes) to the shooter power variables
        Shooter1.setPower(ShooterPower);
        Shooter2.setPower(ShooterPower);
        //Sets Drive motors(NeveRest 40s) to the drive motor power variables
        DriveRight.setPower(RightPower);
        DriveLeft.setPower(LeftPower);

    }

    // This portion of the program scales the input coming in.  In this case, it's taking the input
    // from the Right trigger of gamepad2, and sets it to the closest one stored in a declared array
    // This allows the user to relax their finger a little, and not have to be conscious of holding
    // the trigger down the same way every shot.
    float ScaleShooterRightTrigger(float ScaleInputShooter) {
        // This is an array with the numbers that you can see.  This will allow the right trigger of
        //gamepad 2 to stay the same speed, which leads to more accuracy in consecutive shots.
        double[] ShooterArray = {0, .25, .4, .65, .75, .85};
        //ScaleShooterRightTrigger is multiplied by 7 because arrays start at 0, so its numbers 0-7, instead of 1-8
        int ShooterIndex = (int) (ScaleInputShooter * 5);
        // This allows for "negative" numbers in the array, without having to directly enter them
        if (ShooterIndex < 0) {
            ShooterIndex = -ShooterIndex;
        }
        double ShooterScale = 0;
        if (ScaleInputShooter < 0) {
            ShooterScale = -ShooterArray[ShooterIndex];
        }else {
            ShooterScale = ShooterArray[ShooterIndex];
        }
        return (float)ShooterScale;
    }

    float ScaleShooterLeftTrigger(float ScaleInputShooter) {
        // This is an array with the numbers that you can see.  This will allow the right trigger of
        //gamepad 2 to stay the same speed, which leads to more accuracy in consecutive shots.
        double[] ShooterArray = {0, .65};
        //ScaleShooterRightTrigger is multiplied by 1 because arrays start at 0, so its numbers 0-1, instead of 1-2
        int ShooterIndex = (int) (ScaleInputShooter * 1);
        // This allows for "negative" numbers in the array, without having to directly enter them
        if (ShooterIndex < 0) {
            ShooterIndex = -ShooterIndex;
        }
        double ShooterScale = 0;
        if (ScaleInputShooter < 0) {
            ShooterScale = -ShooterArray[ShooterIndex];
        }else {
            ShooterScale = ShooterArray[ShooterIndex];
        }
        return (float)ShooterScale;
    }
    // This portion of the program "scales" the input coming in.  In this case, it's taking the input
    // from the joysticks of gamepad1, and sets the Left/RightPower to the closest variable stored in
    // a declared array.
    // This allows for prescision driving when moving in the lower half of the 0-1 power scale.
    // While in the higher half, this allows for straighter and more accurate driving.

    float ScaleInputDrive(float ScaleInputDrive) {
        // This is an array with the numbers that you can see.  This will allow both joystick values
        // to be close(not having to be exact) and still be able to get the same power as the other
        // if needed, or if not needed, it can help the driver to be a little more exact while driving.
        double[] DriveArray = {0, .1, .15, .2, .25, .3, .35, .4, .45, .5, .55, .6, .7, .75, .8, .85, .9, 1};
        //ScaleInputDrive is multiplied by 15 because arrays start at 0, so its numbers 0-15, instead of 1-16
        int DriveIndex = (int) (ScaleInputDrive * 17);
        // This allows for "negative" numbers in the array, without having to directly enter them
        if (DriveIndex < 0) {
            DriveIndex = -DriveIndex;
        }
        double DriveScale = 0;
        if (ScaleInputDrive < 0) {
            DriveScale = -DriveArray[DriveIndex];
        }else {
            DriveScale = DriveArray[DriveIndex];
        }
        // Returns the value DriveScale, which is used with the Joysticks when they are set to
        // the variables DriveLeft and DriveRight
        return (float)DriveScale;
    }



}