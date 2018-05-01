package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;


@TeleOp(name="MecanumTeleop", group="TELEOP")
public class MecanumTeleop extends OpMode {
    // This section declares hardware for the program, such as Motors, servos and sensors

    // Declare Motors
    public DcMotor FrontLeft = null;              // NeveRest Orbital 20
    public DcMotor BackLeft = null;               // NeveRest Orbital 20
    public DcMotor FrontRight = null;             // NeveRest Orbital 20
    public DcMotor BackRight = null;              // NeveRest Orbital 20
    public DcMotor ConveyorLeft = null;           // Rev HD Hex Motor
    public DcMotor ConveyorRight = null;          // Rev HD Hex Motor
    public DcMotor DumpingMotor = null;           // NeveRest 60
    public DcMotor LinearSlideMotor = null;       // NeveRest 20

    // Declare Servos
    public CRServo RelicZAxis;                  // Rev SRS
    public Servo JewelArm;                      // Rev SRS
    public Servo RelicClaw;
    public Servo RelicYAxis;
    public Servo CryptoboxServo;
    public Servo FlipperServo;
    //Declare Sensors
    public DistanceSensor FlipperDistance2;
    public DigitalChannel DumperLimitSensorRight;
    public DigitalChannel DumperLimitSensorLeft;
    public BNO055IMU IMU;
    //Variables in use
    boolean RelicYAxisUp = false;
    boolean Dump = false;
    boolean sensorsSeeTwo = false;
    boolean haveGlyphs = false;
    boolean FlipperServoUp = false;
    boolean eitherArePressed = false;
    boolean intake = true;


    double LeftIntake = 1;
    double RightIntake = 1;
    double FlipperServoUpPos = .2;
    double FlipperServoDownPos = 1;
    double LinearSlideSpeed = 0;
    double LinearSlideSpeedMultiplier = 1;
    double RelicYAxisUpPosition = .8;
    double RelicYAxisDownPosition = .31;
    double RelicClawOpenPos = 1;
    double RelicClawClosedPos = .375;
    double StrafingMultiplier = 2;
    double JewelServoUpPos = .55;
    double CryptoboxServoInPos = 0;
    int DumpEncoderOffset = 0;
    double dumpingPower = .5;
    int flipperOffset = 100;

    public ElapsedTime runtime = new ElapsedTime();

    public void init() {
        // This section gets the hardware maps
        FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        FrontRight = hardwareMap.dcMotor.get("FrontRight");
        BackLeft = hardwareMap.dcMotor.get("BackLeft");
        BackRight = hardwareMap.dcMotor.get("BackRight");
        ConveyorLeft = hardwareMap.dcMotor.get("ConveyorLeft");
        ConveyorRight = hardwareMap.dcMotor.get("ConveyorRight");
        DumpingMotor = hardwareMap.dcMotor.get("DumpingMotor");
        LinearSlideMotor = hardwareMap.dcMotor.get("LinearSlideMotor");

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DumpingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DumpingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ConveyorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        ConveyorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        ConveyorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ConveyorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        LinearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        LinearSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Hardware maps for servos
        RelicZAxis = hardwareMap.crservo.get("RelicZAxis");
        JewelArm = hardwareMap.servo.get("JewelServo");
        RelicClaw = hardwareMap.servo.get("RelicClaw");
        RelicYAxis = hardwareMap.servo.get("RelicYAxis");
        CryptoboxServo = hardwareMap.servo.get("CryptoboxServo");
        FlipperServo = hardwareMap.servo.get("FlipperServo");

        LinearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LinearSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Hardware map Sensors
        DumperLimitSensorRight = hardwareMap.get(DigitalChannel.class, "DumperTouchSensorRight");
        DumperLimitSensorLeft = hardwareMap.get(DigitalChannel.class, "DumperTouchSensorLeft");
        DumperLimitSensorRight.setMode(DigitalChannel.Mode.INPUT);
        DumperLimitSensorLeft.setMode(DigitalChannel.Mode.INPUT);
        //IntakeDistance = hardwareMap.get(DistanceSensor.class, "IntakeSensor");
        FlipperDistance2 = hardwareMap.get(DistanceSensor.class, "FlipperSensor2");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        JewelArm.setPosition(JewelServoUpPos);
        CryptoboxServo.setPosition(CryptoboxServoInPos);
        //If intake boolean is true
        if(intake){
            //if gamepad 1b, or gamepad 2 y, reverse intake
            if (gamepad1.b || gamepad2.y) {
                LeftIntake = -1;
                RightIntake = -1;
            //otherwise, intake as normal
            }else{
                LeftIntake = 1;
                RightIntake = 1;
            }
        // If intake is false, stop intake motors
        }else{
            LeftIntake = 0;
            RightIntake = 0;
        }
        //If the distance sensor in the flipper sees something,
        // set the var to let us know we have glyphs
        if(FlipperDistance2.getDistance(DistanceUnit.CM) < 50){
            sensorsSeeTwo = true;
        }else{
            sensorsSeeTwo = false;
        }
        //if gamepad 1 left bumper, start the intake, and put the flipper servo down
        if(gamepad1.left_bumper){
            intake = true;
            FlipperServoUp = false;
        //If GP1 right bumper, stop intake, and put the flipper servo up
        }else if(gamepad1.right_bumper){
            FlipperServoUp = true;
            intake = false;
        }
        //if the flipperServoUp var is true, set the flipper pos to up.  Else, put it down
        if(FlipperServoUp){
            //Servo on flipper up
            FlipperServo.setPosition(FlipperServoUpPos);
        }else{//servo on flipper down
            FlipperServo.setPosition(FlipperServoDownPos);
        }
        //If either of the flipper limit swithces are pressed, set a bool to true, else, false
        if(!DumperLimitSensorRight.getState() || !DumperLimitSensorLeft.getState()){
            //either touch sensors limit switches are pressed
            eitherArePressed = true;
        }else{
            //neither are pressed
            eitherArePressed = false;
        }
        // Start Dumping Code-Helped with flipperOffset By Ethan from 12670 Eclipse

        Dump = gamepad1.right_trigger > .1; //if trigger is pressed down, set dump to true
        if (Dump && eitherArePressed) {//if we want to flip, and the flipper is down
            //set the target pos, and the power to turn to
            DumpingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DumpingMotor.setTargetPosition(DumpingMotor.getCurrentPosition() - (1250+DumpEncoderOffset));
            DumpingMotor.setPower(-dumpingPower);
            intake = false;
        } else if (gamepad1.a) {
            //if we want to adjust the flipper rotation, get the target position and minus
            // (in this case flipperOffest = 100) from the current position
            DumpingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DumpingMotor.setPower(-dumpingPower);
            DumpingMotor.setTargetPosition(DumpingMotor.getTargetPosition() - flipperOffset);
        } else if(gamepad1.y) {
            //if we want to adjust the flipper rotation, get the target position and add
            // (in this case flipperOffest = 100) from the current position
            DumpingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DumpingMotor.setPower(-dumpingPower);
            DumpingMotor.setTargetPosition(DumpingMotor.getTargetPosition() + flipperOffset);
        }else if (!Dump && !eitherArePressed) {
            //if we aren't dumping, and neither are pressed, put the flipper back down
            DumpingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            DumpingMotor.setPower(.25);
            FlipperServoUp = false;
        } else if (eitherArePressed) {
            //if we aren't dumping, and one limit switch is pressed, set flipper power to 0
            DumpingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            DumpingMotor.setPower(0);
            intake = true;
        }
        // End Dumping Code

        // Start Linear Slide/Relic Code
        LinearSlideSpeed = gamepad2.right_stick_y;//this would work to control the slide, but we
        //want to add failsafes (below) to make sure we don't go the wrong way and break something
        if (LinearSlideMotor.getCurrentPosition() < 50) {
            //if the linear slide is closed, don't let the slide go in any further
            LinearSlideSpeed = Range.clip(LinearSlideSpeed, 0, 1);
            LinearSlideMotor.setPower(LinearSlideSpeed*LinearSlideSpeedMultiplier);
        }else{
            //other wise, the slide speed is the right stick value
            LinearSlideSpeed = gamepad2.right_stick_y;
            LinearSlideMotor.setPower(LinearSlideSpeed*LinearSlideSpeedMultiplier);
        }
        //if the linear slide is moving, stop the intake. If we move the slide, it's endgame, and
        // we most likely don't need the intake anymore, so we keep them from using any current.
        //We do include failsafes incase we want to score more glyphs though
        if(Math.abs(LinearSlideMotor.getPower()) > .01){
           intake = false;
        }
        //if GP2 dpad up is pressed, we raise to relic to be able to place over the wall
        //otherwise, if down is pressed, we set it to the down position
        if(gamepad2.dpad_up){
            RelicYAxis.setPosition(RelicYAxisUpPosition);
        }else if(gamepad2.dpad_down){
            RelicYAxis.setPosition(RelicYAxisDownPosition);
        }
        //rotation of the relic arm is controlled like the linear slide, but without limits
        RelicZAxis.setPower(-gamepad2.left_stick_x);
        //a hold-to-use button.  if the GP2 left trigger is held, the relic claw closes.
        // Otherwise, it opens
        if(gamepad2.left_trigger > .1){
            RelicClaw.setPosition(RelicClawClosedPos);
        }else{
            RelicClaw.setPosition(RelicClawOpenPos);
        }
        // End Linear Slide/Relic Code

        // Start Driving Code
        double DrivingMultiplier = 1;
        //this allows for a "slow mode".  The driving multiplier is .35 while the trigger is held,
        //otherwise, the multiplier is 1 (so it's only .35 if the trigger is held).  We use the
        //multiplier down below, where we set speeds of the motor as well as the strafing multiplier
        if(gamepad1.left_trigger > .1){
            DrivingMultiplier = .35;
        }else{
            DrivingMultiplier = 1;
        }

        //Math, trig, it works.  Don't mess with it

        double FrontLeftVal =
                gamepad1.left_stick_y*DrivingMultiplier
                - (gamepad1.left_stick_x*StrafingMultiplier*DrivingMultiplier)
                + -gamepad1.right_stick_x*DrivingMultiplier;
        double FrontRightVal =
                gamepad1.left_stick_y*DrivingMultiplier
                + (gamepad1.left_stick_x*StrafingMultiplier*DrivingMultiplier)
                - -gamepad1.right_stick_x*DrivingMultiplier;
        double BackLeftVal =
                gamepad1.left_stick_y*DrivingMultiplier
                + (gamepad1.left_stick_x*StrafingMultiplier)
                + -gamepad1.right_stick_x*DrivingMultiplier;
        double BackRightVal = gamepad1.left_stick_y*DrivingMultiplier
                - (gamepad1.left_stick_x*StrafingMultiplier*DrivingMultiplier)
                - -gamepad1.right_stick_x*DrivingMultiplier;

        //Move range to between 0 and +1, if not already
        double[] wheelPowers = {FrontRightVal, FrontLeftVal, BackLeftVal, BackRightVal};
        Arrays.sort(wheelPowers);
        if (wheelPowers[3] > 1) {
            FrontLeftVal /= wheelPowers[3];
            FrontRightVal /= wheelPowers[3];
            BackLeftVal /= wheelPowers[3];
            BackRightVal /= wheelPowers[3];
        }
        FrontLeft.setPower(FrontLeftVal);
        FrontRight.setPower(FrontRightVal);
        BackLeft.setPower(BackLeftVal);
        BackRight.setPower(BackRightVal);
        // End Driving Code
        ConveyorLeft.setPower(LeftIntake);
        ConveyorRight.setPower(RightIntake);
    }
}