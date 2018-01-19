package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
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
    public CRServo IntakeServoLeft = null;      // Vex 393
    public CRServo IntakeServoRight = null;     // Vex 393
    public CRServo TopIntakeServoLeft = null;   // Rev SRS
    public CRServo TopIntakeServoRight = null;  // Rev SRS
    public CRServo DumpConveyor = null;         // Rev SRS
    public Servo Blocker;                // Rev SRS  Heh, block-er, cause it keeps glyphs from falling out
    public Servo JewelArm;               // Rev SRS
    public Servo RelicClaw;
    public Servo RelicYAxis;
    public Servo RelicZAxis;
    public Servo CryptoboxServo;

    //Declare Sensors
    public DistanceSensor IntakeDistance;
    public DigitalChannel DumperTouchSensorRight;
    public BNO055IMU IMU;

    // Variables
    int DumpingGearDriven = 40; // Gear connected to dumping motor
    int DumpingGearDriving = 80; // Gear connected to dumping assembly
    int DumpingDegreesOfTravel = 80; // Wanted degrees of the dump to travel
    int FractionOfRevolutionToDump = 360/DumpingDegreesOfTravel;
    int DumpingMotorEncoderTicks = 1680; // NeveRest 60
    int DumpingGearRatio = DumpingGearDriving/DumpingGearDriven; // 2:1
    int DumpingEncoderTicksPerRevolution = DumpingMotorEncoderTicks*DumpingGearRatio;
    int EncoderTicksToDump = DumpingEncoderTicksPerRevolution/FractionOfRevolutionToDump;
    int linearSlideDistance = 8;

    double LinearSlideSpeed = 0;
    double LinearSlideSpeedMultiplier = 1;
    double RelicYAxisUpPosition = .8;
    double RelicYAxisDownPosition = .31;;
    double RelicClawOpenPos = .8;
    double RelicClawClosedPos = .15;
    double StrafingMultiplier = 2;
    double BlockerServoUp = .35;
    double BlockerServoDown = .56;
    double JewelServoUpPos = .60;
    double JoystickMultiplier = 1; // variable to allow slower driving speeds
    double IntakeSpeed = -.7;
    double CryptoboxServoInPos = 0;
    double CryptoboxServoOutPos = 1;

    boolean ClawOpen = true;
    boolean Dump = false;
    boolean Intake = false;
    boolean BlockerUp = true;
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
        DumpingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ConveyorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        ConveyorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        LinearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Hardware maps for servos
        IntakeServoLeft = hardwareMap.crservo.get("IntakeServoLeft");
        IntakeServoRight = hardwareMap.crservo.get("IntakeServoRight");
        TopIntakeServoLeft = hardwareMap.crservo.get("TopIntakeServoLeft");
        TopIntakeServoRight = hardwareMap.crservo.get("TopIntakeServoRight");
        DumpConveyor = hardwareMap.crservo.get("DumpConveyor");
        Blocker = hardwareMap.servo.get("Blocker");
        JewelArm = hardwareMap.servo.get("JewelServo");
        RelicClaw = hardwareMap.servo.get("RelicClaw");
        RelicYAxis = hardwareMap.servo.get("RelicYAxis");
        RelicZAxis = hardwareMap.servo.get("RelicZAxis");
        CryptoboxServo = hardwareMap.servo.get("CryptoboxServo");

        IntakeServoLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        IntakeServoRight.setDirection(DcMotorSimple.Direction.REVERSE);
        TopIntakeServoLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        TopIntakeServoRight.setDirection(DcMotorSimple.Direction.REVERSE);
        DumpConveyor.setDirection(DcMotorSimple.Direction.FORWARD);
        LinearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LinearSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Hardware map Sensors
        DumperTouchSensorRight = hardwareMap.get(DigitalChannel.class, "DumperTouchSensorRight");
        DumperTouchSensorRight.setMode(DigitalChannel.Mode.INPUT);
        IntakeDistance = hardwareMap.get(DistanceSensor.class, "IntakeSensor");
    }

    @Override
    public void init_loop() {
        Blocker.setPosition(BlockerServoUp);
        JewelArm.setPosition(JewelServoUpPos);

    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        JewelArm.setPosition(JewelServoUpPos);
        CryptoboxServo.setPosition(CryptoboxServoInPos);

        // Start Intake Code
        if (gamepad1.a) {
            Intake = !Intake;
        }
        if(Intake) {
            ConveyorLeft.setPower(0);
            ConveyorRight.setPower(0);
            TopIntakeServoLeft.setPower(0);
            TopIntakeServoRight.setPower(0);
            Intake = false;
            Intake = false;
        }else{
            double SensorVal = IntakeDistance.getDistance(DistanceUnit.CM);
            if (SensorVal <= 9) {
                IntakeServoLeft.setPower(IntakeSpeed);
                IntakeServoRight.setPower(-IntakeSpeed);
            }else if(SensorVal > 9 && SensorVal < 20){
                IntakeServoLeft.setPower(IntakeSpeed);
                IntakeServoRight.setPower(IntakeSpeed);
            }else if (SensorVal >= 20){
                IntakeServoLeft.setPower(-IntakeSpeed);
                IntakeServoRight.setPower(-IntakeSpeed);
            }else{
                IntakeServoLeft.setPower(IntakeSpeed);
                IntakeServoRight.setPower(-IntakeSpeed);
            }
            ConveyorLeft.setPower(1);
            ConveyorRight.setPower(1);
            TopIntakeServoLeft.setPower(1);
            TopIntakeServoRight.setPower(1);
        }
        if(gamepad2.left_bumper){
            TopIntakeServoRight.setPower(-1);
        }
        // End Intake and Conveyor code

        // Start Dumping Code
        Dump = gamepad2.right_trigger > .1;
        double dumpingPower = .25;
        if (Dump && !DumperTouchSensorRight.getState()) {
            DumpingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DumpingMotor.setTargetPosition(DumpingMotor.getCurrentPosition() - EncoderTicksToDump);
            DumpingMotor.setPower(-dumpingPower);
            BlockerUp = false;
        } else if (Dump) {
            DumpingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DumpingMotor.setPower(-dumpingPower);
            BlockerUp = false;
        } else if (!Dump && DumperTouchSensorRight.getState()) {
            DumpingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            DumpingMotor.setPower(dumpingPower);
            BlockerUp = false;
        } else if (!DumperTouchSensorRight.getState() && Math.abs(DumpConveyor.getPower()) > .5) {
            DumpingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BlockerUp = false;
        }else if (!DumperTouchSensorRight.getState()) {
            DumpingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BlockerUp = true;
        }

        if (gamepad1.right_bumper || gamepad2.right_bumper ) {
            DumpConveyor.setPower(1);
        } else if(Math.abs(gamepad2.left_stick_x) > .1){
            DumpConveyor.setPower(Range.clip(gamepad2.left_stick_y, -1, 1));
        }else {
            DumpConveyor.setPower(.25);
        }

        if (gamepad2.a || !BlockerUp || DumpConveyor.getPower() > .5) {
            Blocker.setPosition(BlockerServoDown);
        } else{
            Blocker.setPosition(BlockerServoUp);
        }
        // End Dumping Code

        // Start Linear Slide/Relic Code
        if(gamepad2.left_trigger > .1){
            LinearSlideSpeedMultiplier = .15;
            LinearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }else{
            LinearSlideSpeedMultiplier = 1;
            LinearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        LinearSlideSpeed = gamepad2.right_stick_y;
        if (LinearSlideMotor.getCurrentPosition() >= 2600){
            LinearSlideSpeed = Range.clip(LinearSlideSpeed, -1, 0);
            LinearSlideMotor.setPower(LinearSlideSpeed*LinearSlideSpeedMultiplier);
        } else if (LinearSlideMotor.getCurrentPosition() < 50) {
            LinearSlideSpeed = Range.clip(LinearSlideSpeed, 0, 1);
            LinearSlideMotor.setPower(LinearSlideSpeed*LinearSlideSpeedMultiplier);
        }else{
            LinearSlideSpeed = gamepad2.right_stick_y;
            LinearSlideMotor.setPower(LinearSlideSpeed*LinearSlideSpeedMultiplier);
        }

        if(gamepad1.dpad_left || gamepad2.dpad_left){
            double CurrentPos = RelicZAxis.getPosition();
            RelicZAxis.setPosition(CurrentPos + .025);
        }else if(gamepad1.dpad_right || gamepad2.dpad_right){
            double CurrentPos = RelicZAxis.getPosition();
            RelicZAxis.setPosition(CurrentPos - .025);
        }else{
            double CurrentPos = RelicZAxis.getPosition();
            RelicZAxis.setPosition(CurrentPos);
        }

        if(gamepad2.dpad_up){
            RelicYAxis.setPosition(RelicYAxisUpPosition);
        }else if(gamepad2.dpad_down){
            RelicYAxis.setPosition(RelicYAxisDownPosition);
        }else{
            double CurrentPos;
            CurrentPos = RelicYAxis.getPosition();
            RelicYAxis.setPosition(CurrentPos);
        }
        if(gamepad2.b){
            ClawOpen = !ClawOpen;
        }
        if(ClawOpen){
            RelicClaw.setPosition(RelicClawOpenPos);
        }else if(!ClawOpen){
            RelicClaw.setPosition(RelicClawClosedPos);
        }
        // End Linear Slide/Relic Code

        // Start Driving Code
        if (gamepad1.left_trigger > .1) {
            JoystickMultiplier = .1;
            StrafingMultiplier = 2;
        } else if (gamepad1.right_trigger > .1){
            JoystickMultiplier = .4;
            StrafingMultiplier = 2;
        } else{
            JoystickMultiplier = 1;
            StrafingMultiplier = 2;
        }
        double FrontLeftVal =
                gamepad1.left_stick_y
                + (gamepad1.left_stick_x*StrafingMultiplier)
                + -gamepad1.right_stick_x;
        double FrontRightVal =
                gamepad1.left_stick_y
                - (gamepad1.left_stick_x*StrafingMultiplier)
                - -gamepad1.right_stick_x;
        double BackLeftVal =
                gamepad1.left_stick_y
                - (gamepad1.left_stick_x*StrafingMultiplier)
                + -gamepad1.right_stick_x;
        double BackRightVal =
                gamepad1.left_stick_y
                + (gamepad1.left_stick_x*StrafingMultiplier)
                - -gamepad1.right_stick_x;

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

        telemetry.addData("LinearSlide Pos", LinearSlideMotor.getCurrentPosition());
        telemetry.update();
    }
}