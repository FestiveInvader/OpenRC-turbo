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
    public Servo Blocker;                       // Rev SRS  Heh, block-er, cause it keeps glyphs from falling out
    public Servo JewelArm;                      // Rev SRS
    public Servo RelicClaw;
    public Servo RelicYAxis;
    public Servo CryptoboxServo;
    public Servo IntakeServo;
    public Servo FlipperServo;
    //Declare Sensors
    public DistanceSensor IntakeDistance;
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
    double IntakeServoUp = 1;
    double IntakeServo90Pos = .7;
    double IntakeServoDown = 0;
    double glyphsSeenTime;
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
    int intakeValLeft = 22;



    //variables out of use, but maybe needed in future use
    boolean BlockerUp = true;
    boolean glyphIn = false;
    boolean placedGlyphs = false;
    double BlockerServoUp = .35;
    double BlockerServoDown = .56;
    double JoystickMultiplier = 1;
    double MidIntakeSpeed = -.7;
    int intakeValRight = 20;

    // Variables
    /*int DumpingGearDriven = 40; // Gear connected to dumping motor
    int DumpingGearDriving = 80; // Gear connected to dumping assembly
    int DumpingDegreesOfTravel = 122; // Wanted degrees of the dump to travel
    int FractionOfRevolutionToDump = 360/DumpingDegreesOfTravel;
    int DumpingMotorEncoderTicks = 1680; // NeveRest 60
    int DumpingGearRatio = DumpingGearDriving/DumpingGearDriven; // 2:1
    int DumpingEncoderTicksPerRevolution = DumpingMotorEncoderTicks*DumpingGearRatio;
    int EncoderTicksToDump = DumpingEncoderTicksPerRevolution/FractionOfRevolutionToDump;*/

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
        Blocker = hardwareMap.servo.get("Blocker");
        JewelArm = hardwareMap.servo.get("JewelServo");
        RelicClaw = hardwareMap.servo.get("RelicClaw");
        RelicYAxis = hardwareMap.servo.get("RelicYAxis");
        CryptoboxServo = hardwareMap.servo.get("CryptoboxServo");
        IntakeServo = hardwareMap.servo.get("IntakeServo");
        FlipperServo = hardwareMap.servo.get("FlipperServo");

        LinearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LinearSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Hardware map Sensors
        DumperLimitSensorRight = hardwareMap.get(DigitalChannel.class, "DumperTouchSensorRight");
        DumperLimitSensorLeft = hardwareMap.get(DigitalChannel.class, "DumperTouchSensorLeft");
        DumperLimitSensorRight.setMode(DigitalChannel.Mode.INPUT);
        DumperLimitSensorLeft.setMode(DigitalChannel.Mode.INPUT);
        //IntakeDistance = hardwareMap.get(DistanceSensor.class, "IntakeSensor");
        IntakeDistance = hardwareMap.get(DistanceSensor.class, "FlipperSensor1");
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
        if(intake){
            if (gamepad1.b || gamepad2.y) {
                LeftIntake = -1;
                RightIntake = -1;
            }else{
                LeftIntake = 1;
                RightIntake = 1;
                /*double SensorVal = IntakeDistance.getDistance(DistanceUnit.CM);
                if (SensorVal <= intakeValLeft && SensorVal > 6) {
                    LeftIntake = 1;
                    RightIntake = 1;
                }else if(SensorVal > intakeValLeft){
                    LeftIntake = 0;
                    RightIntake = 1;
                }else if (SensorVal <= 5.5) {
                    LeftIntake = 1;
                    RightIntake = 0;
                }else {
                    LeftIntake = 1;
                    RightIntake = 1;
                }*/
            }
        }else{
            LeftIntake = 0;
            RightIntake = 0;
        }



        if(FlipperDistance2.getDistance(DistanceUnit.CM) < 50){
            sensorsSeeTwo = true;
        }else{
            sensorsSeeTwo = false;
        }

        if(gamepad1.left_bumper){
            intake = true;
            //Put everything done
            //haveGlyphs = false;
            FlipperServoUp = false;
        }else if(gamepad1.right_bumper){
            //start the putting up sequence
            FlipperServoUp = true;
            intake = false;
            /*haveGlyphs = true;
            glyphsSeenTime = runtime.seconds();*/
        }

       /* if(haveGlyphs && runtime.seconds() - glyphsSeenTime > .1 && runtime.seconds() - glyphsSeenTime < .35) {
            IntakeServo.setPosition(IntakeServoUp);
            telemetry.addData("Stuck in 1 loop", 1);
        }else if(haveGlyphs && runtime.seconds() - glyphsSeenTime > .35 && runtime.seconds() - glyphsSeenTime < .55){
            FlipperServoUp = true;
            telemetry.addData("Stuck in 2 loop", 2);
            //when we have the glyphs and time is between /75 and .85 seconds
        }else if(haveGlyphs && runtime.seconds() - glyphsSeenTime > .55){
            telemetry.addData("Stuck in 3 loop", 3);
            //Same as above, but a little later in time
            IntakeServo.setPosition(IntakeServo90Pos);
            FlipperServoUp = true;
        }
        if(!haveGlyphs){
            IntakeServo.setPosition(IntakeServoDown);
            FlipperServoUp = false;
            telemetry.addData("in the !haveglyphs", 1);
        }*/

        if(FlipperServoUp){
            //Servo on flipper up
            FlipperServo.setPosition(FlipperServoUpPos);
        }else{//servo on flipper down
            FlipperServo.setPosition(FlipperServoDownPos);
        }
            // Start Intake Code

        // End Intake and Conveyor code
        if(!DumperLimitSensorRight.getState() || !DumperLimitSensorLeft.getState()){
            //either touch sensors limit switches are pressed
            eitherArePressed = true;
        }else{
            eitherArePressed = false;
        }
        // Start Dumping Code
        Dump = gamepad1.right_trigger > .1;
        double dumpingPower = .5;
        //Helped with flipperOffset By Ethan from 12670 Eclipse
        int flipperOffset = 100;
        if (Dump && eitherArePressed) {
            DumpingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DumpingMotor.setTargetPosition(DumpingMotor.getCurrentPosition() - (1250+DumpEncoderOffset));
            DumpingMotor.setPower(-dumpingPower);
            intake = false;
        } else if (gamepad1.a) {
            DumpingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DumpingMotor.setPower(-dumpingPower);
            DumpingMotor.setTargetPosition(DumpingMotor.getTargetPosition() - flipperOffset);
        } else if(gamepad1.y) {
            DumpingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DumpingMotor.setPower(-dumpingPower);
            DumpingMotor.setTargetPosition(DumpingMotor.getTargetPosition() + flipperOffset);
        }else if (!Dump && !eitherArePressed) {
            DumpingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            DumpingMotor.setPower(.25);
            FlipperServoUp = false;

        } else if (eitherArePressed) {
            DumpingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            DumpingMotor.setPower(0);
            intake = true;

        }
        // End Dumping Code

        // Start Linear Slide/Relic Code

        LinearSlideSpeed = gamepad2.right_stick_y;
        if (LinearSlideMotor.getCurrentPosition() < 50) {
            LinearSlideSpeed = Range.clip(LinearSlideSpeed, 0, 1);
            LinearSlideMotor.setPower(LinearSlideSpeed*LinearSlideSpeedMultiplier);
        }else{
            LinearSlideSpeed = gamepad2.right_stick_y;
            LinearSlideMotor.setPower(LinearSlideSpeed*LinearSlideSpeedMultiplier);
        }
        if(Math.abs(LinearSlideMotor.getPower()) > .01){
           intake = false;
        }

        RelicZAxis.setPower(-gamepad2.left_stick_x);

        if(gamepad2.dpad_up){
            RelicYAxisUp = true;
        }else if(gamepad2.dpad_down){
            RelicYAxisUp = false;
        }
        if(RelicYAxisUp){
            RelicYAxis.setPosition(RelicYAxisUpPosition);
        }else{
            RelicYAxis.setPosition(RelicYAxisDownPosition);
        }
        //Right joystick for relic claw rotation
        //left bumper super secret switcher


        if(gamepad2.left_trigger > .1){
            RelicClaw.setPosition(RelicClawClosedPos);
        }else{
            RelicClaw.setPosition(RelicClawOpenPos);
        }
        // End Linear Slide/Relic Code

        // Start Driving Code
        double DrivingMultiplier = 1;
        if(gamepad1.left_trigger > .1){
            DrivingMultiplier = .35;
        }else{
            DrivingMultiplier = 1;
        }
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
        telemetry.addData("Intake Distance", IntakeDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Flippersensor2", FlipperDistance2.getDistance(DistanceUnit.CM));
        telemetry.addData("FlipperTouchLeft", DumperLimitSensorLeft.getState());
        telemetry.addData("FlipperTouchRight", DumperLimitSensorRight.getState());
        telemetry.addData("SensorSeeTwo", sensorsSeeTwo);
        telemetry.addData("Clamp glyphs", FlipperServoUp);
        telemetry.addData("have gylphs", haveGlyphs);
        telemetry.addData("LSlide Pos", LinearSlideMotor.getCurrentPosition());
        telemetry.update();
        }
}