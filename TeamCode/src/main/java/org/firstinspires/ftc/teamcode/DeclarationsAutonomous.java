package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Arrays;

import static java.lang.System.currentTimeMillis;

//@Author Eric Adams, Team 8417 'Lectric Legends

public class DeclarationsAutonomous extends LinearOpMode {
    // This section declares hardware for the program, such as Motors, servos and sensors
    // Declare Motors
    public DcMotor FrontLeft = null;              // NeveRest Orbital 20
    public DcMotor BackLeft = null;               // NeveRest Orbital 20
    public DcMotor FrontRight = null;             // NeveRest Orbital 20
    public DcMotor BackRight = null;              // NeveRest Orbital 20
    public DcMotor ConveyorLeft = null;   // Rev HD Hex Motor
    public DcMotor ConveyorRight = null;  // Rev HD Hex Motor
    public DcMotor DumpingMotor = null;           // NeveRest 60

    // Declare Servos
    public CRServo IntakeServoLeft = null;  // VEX 393
    public CRServo IntakeServoRight = null; // VEX 393
    public CRServo TopIntakeServoLeft = null;
    public CRServo TopIntakeServoRight = null;
    public CRServo DumpConveyor = null;     // Rev SRS
    public Servo Blocker = null;            // Rev SRS  Heh, block-er
    public Servo JewelArm = null;           // Rev SRS
    public Servo CryptoboxServo = null;

    public ModernRoboticsI2cRangeSensor BackDistance;
    public ModernRoboticsI2cRangeSensor RightDistance;
    public DistanceSensor IntakeDistance;
    public DistanceSensor ConveyorDistance;
    public DistanceSensor CryptoboxDistance;
    public ColorSensor ConveyorColor;
    public ColorSensor JewelColor;
    public DigitalChannel DumperTouchSensorRight;
    public BNO055IMU IMU;

    // Variables used  in functions
    double CountsPerRev = 537.6;    // Andymark NeveRest 40
    double GearRatio = .88888888888888889;
    double WheelDiameterInches = 4.0;     // For figuring circumference
    double CountsPerInch = ((CountsPerRev / (WheelDiameterInches * 3.1415))*GearRatio);
    double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    double P_TURN_COEFF = .2;     // Larger is more responsive, but also less stable
    double P_DRIVE_COEFF = .15;     // Larger is more responsive, but also less stable
    int[] CurrentCryptoBox = new int[]{0,0,0,0,0,0,0,0,0,0,0,0};
    double cryptoboxHeading = 0;
    int Forward = 1;
    int Reverse = -1;
    int RelicSide = 1;
    int TeamSide = 2;

    int DumpingGearDriven = 40; // Gear connected to dumping motor
    int DumpingGearDriving = 80; // Gear connected to dumping assembly
    int DumpingDegreesOfTravel = 85; // Wanted degrees of the dump to travel
    int FractionOfRevolutionToDump = 360/DumpingDegreesOfTravel;
    int DumpingMotorEncoderTicks = 1680; // NeveRest 60
    int DumpingGearRatio = DumpingGearDriving/DumpingGearDriven; // 2:1
    int DumpingEncoderTicksPerRevolution = DumpingMotorEncoderTicks*DumpingGearRatio;
    int EncoderTicksToDump = DumpingEncoderTicksPerRevolution/FractionOfRevolutionToDump;


    double BlockerServoUp = .35;
    double BlockerServoDown = .56;
    double JewelServoUpPos = .61;
    double JewelServoDistancePos = .34;
    double JewelServoDownPos = .14; //.2 really
    double RegularTurnSpeed = .165;
    double IntakeSpeed = 1;
    double CryptoboxServoInPos = .1;
    double CryptoboxServoOutPos = .985;
    double CryptoboxServoMidPos = .5;

    VuforiaLocalizer vuforia;
    RelicRecoveryVuMark CryptoKey;
    public ElapsedTime runtime = new ElapsedTime();

    float d = 0;
    float denom = 0;
    float num = 0;
    int loop_counter = 0;
    int previous_power = 0;
    int count_list[]=new int[1];
    long time_list[]=new long[1];
    long PreviousTime = 0;
    @Override
    public void runOpMode() {
        // This section gets the hardware maps
        FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        FrontRight = hardwareMap.dcMotor.get("FrontRight");
        BackLeft = hardwareMap.dcMotor.get("BackLeft");
        BackRight = hardwareMap.dcMotor.get("BackRight");
        ConveyorLeft = hardwareMap.dcMotor.get("ConveyorLeft");
        ConveyorRight = hardwareMap.dcMotor.get("ConveyorRight");
        DumpingMotor = hardwareMap.dcMotor.get("DumpingMotor");

        DumpingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ConveyorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        ConveyorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Hardware maps for servos
        IntakeServoLeft = hardwareMap.crservo.get("IntakeServoLeft");
        IntakeServoRight = hardwareMap.crservo.get("IntakeServoRight");
        TopIntakeServoLeft = hardwareMap.crservo.get("TopIntakeServoLeft");
        TopIntakeServoRight = hardwareMap.crservo.get("TopIntakeServoRight");
        DumpConveyor = hardwareMap.crservo.get("DumpConveyor");
        Blocker = hardwareMap.servo.get("Blocker");
        JewelArm = hardwareMap.servo.get("JewelServo");
        CryptoboxServo = hardwareMap.servo.get("CryptoboxServo");

        IntakeServoLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        IntakeServoRight.setDirection(DcMotorSimple.Direction.REVERSE);
        TopIntakeServoLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        TopIntakeServoRight.setDirection(DcMotorSimple.Direction.REVERSE);
        DumpConveyor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Initialize and hardware map Sensors
        DumperTouchSensorRight = hardwareMap.get(DigitalChannel.class, "DumperTouchSensorRight");
        DumperTouchSensorRight.setMode(DigitalChannel.Mode.INPUT);
        IntakeDistance = hardwareMap.get(DistanceSensor.class, "IntakeSensor");
        ConveyorDistance = hardwareMap.get(DistanceSensor.class, "ConveyorSensor");
        CryptoboxDistance = hardwareMap.get(DistanceSensor.class, "CryptoboxSensor");
        RightDistance = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "RightDistance");
        BackDistance = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "BackDistance");
        ConveyorColor = hardwareMap.get(ColorSensor.class, "ConveyorSensor");
        JewelColor = hardwareMap.get(ColorSensor.class, "JewelSensor");

        // Start Init IMU
        BNO055IMU.Parameters Bparameters = new BNO055IMU.Parameters();
        Bparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        Bparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        Bparameters.calibrationDataFile = "BNO055IMUCalibration.json";
        Bparameters.loggingEnabled = true;
        Bparameters.loggingTag = "IMU";
        Bparameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        IMU = hardwareMap.get(BNO055IMU.class, "IMU");
        IMU.initialize(Bparameters);
        // End Init IMU
        telemetry.addData("IMU Init'd", true);
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // OR...  Do Not Activate the Camera Monitor View, to save power
        parameters.vuforiaLicenseKey = "ASW6AVr/////AAAAGcNlW86HgEydiJgfyCjQwxJ8z/aUm0uGPANypQfjy94MH3+UHpB" +
                "60bep2E2CIpQCtDevYkE3I9xx1nrU3d9mxfoelhGARuvw7GBwTSjMG0GDQbuSgWGZ1X1IVW35MjOoeg57y/IJGCosxEGz" +
                "J0VHTFmKLkPoGCHQysZ2M2d8AVQDyG+PobNjbYQeC16TZJ7SJyXHr7MJxpj/MKbRwb/bZ1icAvWdrNWiB48dyRjIESk7MewD" +
                "X5ke8X6KEjZkKFiQxbAeCbh3DoxTXVJujcSHAdzncIsFIxLqvh5pX0il9tX+vs+64CUjEbi/HaES7S0q3d3MrVQMCXz77zynqv" +
                "iei9O/4BmYcLw6W7c+Es0sClX/";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        telemetry.addData("Init'd VuForia ", 1);
        telemetry.update();
        relicTrackables.activate();
        CryptoKey = RelicRecoveryVuMark.UNKNOWN;

        Blocker.setPosition(BlockerServoUp);
        JewelArm.setPosition(JewelServoUpPos);
        while(!isStarted()){
            telemetry.addData("Ready to start", CryptoKey);
            telemetry.update();
        }
        runtime.reset();
        while(CryptoKey == RelicRecoveryVuMark.UNKNOWN && runtime.seconds() < 5){
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                CryptoKey = vuMark;
                telemetry.addData("VuMark", "%s visible", vuMark);
                telemetry.addData("int val", CryptoKey);
            }
            else {
                telemetry.addData("VuMark", "not visible");
            }
            telemetry.update();
        }
    }

    // Start Movement methods
    public void drive(double YSpeed, double XSpeed, double time){
       double timeStarted = runtime.time();
       FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

       while(opModeIsActive() && runtime.time() - timeStarted < time) {
           moveBy(YSpeed, XSpeed, 0);
           telemetry.addData("TargetPos", time);
           telemetry.addData("CurrentPos", FrontLeft.getCurrentPosition());
           telemetry.update();
       }
        stopDriveMotors();
    }
    public void EncoderDrive(double speed, double leftInches, double rightInches, int direction) {
        int NewLeftTarget;
        int NewRightTarget;
        boolean Running = true;
        double Speed = speed;


        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        if (Running) {
            // Determine new target position, and pass to motor controller
            // Calculates the needed encoder ticks by multiplying a pre-determined amount of CountsPerInches,
            // and the method input gets the actual distance travel in inches
            NewLeftTarget = FrontLeft.getCurrentPosition() + (int) (leftInches * CountsPerInch * direction);
            NewRightTarget = FrontRight.getCurrentPosition() + (int) (rightInches * CountsPerInch * direction);
            // Gives the encoders the target.

            // This gets the absolute value of the encoder positions at full speed - the current speed, and while it's greater than 0, it will continues increasing the speed.
            // This allows the robot to accelerate over a set number of inches, which reduces wheel slippage and increases overall reliability
            while (opModeIsActive() && Running) {// && opModeIsActive

                if(Math.abs(NewLeftTarget) - Math.abs(FrontLeft.getCurrentPosition()) < -1) {
                    //If absolute value of wanted encoder count at finish -
                    // the absolute value of current motor is < -1, then stop running.
                    Running = false;
                }
                moveBy(.2*direction, 0,0);
                telemetry.addData("Runnig", FrontLeft.getCurrentPosition());
                telemetry.addData("Runnig", FrontRight.getCurrentPosition());
                telemetry.addData("Runnig", BackLeft.getCurrentPosition());
                telemetry.addData("Runnig", BackRight.getCurrentPosition());
                telemetry.update();
            }

            // Stops all motion
            // Set to run without encoder, so it's not necessary to declare this every time after the method is used
            FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // Set power to 0
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);
        }
    }
    public void EncoderTurn(double speed, double leftInches, double rightInches, int Direction) {
        // Declares variables that are used for this method
        int NewLeftTarget;
        int NewRightTarget;
        double MinSpeed = .2;
        boolean Running = true;
        double Speed = speed;

        // Resets encoders to 0
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Checks to make sure that encoders are reset.
        while(FrontLeft.getCurrentPosition() > 1 || FrontRight.getCurrentPosition() > 1 || BackLeft.getCurrentPosition() > 1 || BackRight.getCurrentPosition()> 1){
            sleep(15);
        }

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        if (Running) {
            // Determine new target position, and pass to motor controller
            // Calculates the needed encoder ticks by multiplying a pre-determined amount of CountsPerInches,
            // and the method input gets the actual distance travel in inches
            NewLeftTarget = FrontRight.getCurrentPosition() + (int) (leftInches * CountsPerInch);
            NewRightTarget = FrontRight.getCurrentPosition() + (int) (rightInches * CountsPerInch);
            // Gives the encoders the target.
            FrontLeft.setTargetPosition(NewLeftTarget);
            FrontRight.setTargetPosition(NewRightTarget);
            BackLeft.setTargetPosition(NewLeftTarget);
            BackRight.setTargetPosition(NewRightTarget);


            // Turn On RUN_TO_POSITION
            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // This gets the absolute value of the encoder positions at full speed - the current speed, and while it's greater than 0, it will continues increasing the speed.
            // This allows the robot to accelerate over a set number of inches, which reduces wheel slippage and increases overall reliability
            while (opModeIsActive() && Running) {// && opModeIsActive

                if(Math.abs(NewLeftTarget) - Math.abs(FrontLeft.getCurrentPosition()) < -1) {
                    //If absolute value of wanted encoder count at finish -
                    // the absolute value of current motor is < -1, then stop running.
                    Running = false;
                }
                FrontLeft.setPower(Range.clip(Math.abs(Speed), MinSpeed*Direction,Direction*speed));
                FrontRight.setPower(Range.clip(Math.abs(Speed), MinSpeed*Direction,Direction*speed));
                BackLeft.setPower(Range.clip(Math.abs(Speed), MinSpeed*Direction,Direction*speed));
                BackRight.setPower(Range.clip(Math.abs(Speed), MinSpeed*Direction,Direction*speed));
                telemetry.addData("Runnig", 10);
            }

            // Stops all motion
            // Set to run without encoder, so it's not necessary to declare this every time after the method is used
            FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // Set power to 0
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);

        }
    }
    public void gyroTurn ( double speed, double angle) {
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, -angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;
        double PosOrNeg = 1;
        double SpeedError;
        double error = getError(angle);
        // determine turn power based on +/- error
        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0;
            rightSpeed = 0;
            onTarget = true;
        }
        else {
            // This is the main part of the Proportional GyroTurn.  This takes a set power variable,
            // and adds the absolute value of error/150.  This allows for the robot to turn faster when farther
            // away from the target, and turn slower when closer to the target.  This allows for quicker, as well
            // as more accurate turning when using the GyroSensor
            PosOrNeg = Range.clip((int)error, -1, 1);
            steer = getSteer(error, PCoeff);
            leftSpeed  = Range.clip(speed + Math.abs(error/150) , speed, .5)* PosOrNeg;

            rightSpeed = -leftSpeed;
        }

        // Set motor speeds.
        FrontLeft.setPower(leftSpeed);
        FrontRight.setPower(rightSpeed);
        BackLeft.setPower(leftSpeed);
        BackRight.setPower(rightSpeed);
        // Display debug info in telemetry.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Left,Right.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
        return onTarget;
    }
    public double getError(double targetAngle) {

        double robotError;
        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getHeading();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * (2*PCoeff), -1, 1);
    }
    public double getHeading(){
        Orientation angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
    public void moveBy(double y, double x, double c) {
        double FrontLeftVal = -y  +x + c;
        double FrontRightVal = -y - x - c;
        double BackLeftVal = -y - x + c;
        double BackRightVal = -y + x - c;

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

    }
    public void fieldOriented(double y, double x, double c, double gyroheading) {
        // X and Y are left joy, C is rotation of right joy
        double cosA = Math.cos(Math.toRadians(gyroheading));
        double sinA = Math.sin(Math.toRadians(gyroheading));
        double xOut = x * cosA - y * sinA;
        double yOut = x * sinA + y * cosA;
        moveBy(yOut, xOut, c);
    }
    public void stopDriveMotors(){
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
    }
    public void findWall(double speed, int Distance){
        JewelArm.setPosition(JewelServoDistancePos);
        sleep(500);
        while(opModeIsActive() && BackDistance.getDistance(DistanceUnit.CM) > Distance){
            moveBy(speed, 0, 0);
        }
        JewelArm.setPosition(JewelServoUpPos);
        stopDriveMotors();
    }
    public void driveAndPlace(RelicRecoveryVuMark CryptoKey, int Direction, int Placement, double gyroOffset){
        // Tolerance +- of the beginning distance, to account for small mistakes when setting robot up
        // and while knocking the jewel off
        int Tolerance = 4;
        // PylonsToFind controls our while loop, as well as lets us know how many more pylons there are
        int PylonsToFind = 0;
        double BeginningDistance = RightDistance.getDistance(DistanceUnit.CM);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // This allows us to use one function for all autonomous programs, since this covers all
        // cases of use.
        // If we travel forward to get to the cryptobox.  Since the distance sensor is on
        // the right, this means we can set values based on which direction we're going
        if (CryptoKey.equals(RelicRecoveryVuMark.LEFT) && Direction == Forward) {
            PylonsToFind = 3;
        } else if (CryptoKey.equals(RelicRecoveryVuMark.CENTER) && Direction == Forward) {
            PylonsToFind = 2;
        } else if (CryptoKey.equals(RelicRecoveryVuMark.RIGHT) && Direction == Forward) {
            PylonsToFind = 1;
        } else if(CryptoKey.equals(RelicRecoveryVuMark.UNKNOWN)&& Direction == Forward){
            // No target, Vumark failed recognition.  Put in Near column
            PylonsToFind = 1;
        } else if (CryptoKey.equals(RelicRecoveryVuMark.LEFT) && Direction == Reverse) {
            // Else, we're traveling backwards, which means we are on the blue alliance
            PylonsToFind = 0;
        } else if (CryptoKey.equals(RelicRecoveryVuMark.CENTER) && Direction == Reverse) {
            PylonsToFind = 1;
        } else if (CryptoKey.equals(RelicRecoveryVuMark.RIGHT) && Direction == Reverse) {
            PylonsToFind = 2;
        } else if(CryptoKey.equals(RelicRecoveryVuMark.UNKNOWN)&& Direction == Reverse){
            // No target, Vumark failed recognition.  Put in Near column
            PylonsToFind = 0;
        }
        telemetry.addData("PylonsToFind", PylonsToFind);
        telemetry.update();
        //if(RightDistance.getDistance(DistanceUnit.CM) > 0) {
        boolean foundPylon = false;
        while (opModeIsActive() && !foundPylon) {
            // If the current distance is within a certain tolerance margin of which pylon values
            // would extend out of, then drive.  Else, we know we've found a pylon
            if(RightDistance.getDistance(DistanceUnit.CM) > BeginningDistance - Tolerance
                    && RightDistance.getDistance(DistanceUnit.CM) < BeginningDistance + Tolerance) {
                moveBy(.15 * Direction, 0, 0);
            }else{
                foundPylon = true;
            }
            // if there's more than one pylon left to find, or basically, if this is the last one
            // till the target column

            //make run into cryptobox
        }
        int DistanceToTravel = 6*PylonsToFind;
        EncoderDrive(.15, DistanceToTravel, DistanceToTravel, Direction);
        if(Direction == Forward){
            EncoderDrive(.15, 7, 7, Forward);
        }else{
            //EncoderDrive(.15, 3,3, Reverse);
        }
        stopDriveMotors();
        if(Placement == RelicSide) {
            gyroTurn(.215, (-90) + gyroOffset);
        }else if (Direction == Reverse){
            gyroTurn(.215, 0 + gyroOffset);
        }else{
            gyroTurn(.215, 180 + gyroOffset);
        }
        //Function that goes to the wall until a range sensor gets a value of < wanted wall distance
        //CryptoboxServo.setPosition(CryptoboxServoMidPos);
        JewelArm.setPosition(JewelServoDistancePos);
        sleep(500);
        findWall(-.25, 44);
        /*EncoderDrive(.15, 6,6, Reverse);
        EncoderDrive(.05, 1.25,1.25, Forward);*/
        findColumn();
        stopDriveMotors();
        //Function that figures out where to place the glyph (dump, or just use the conveyor)
        //If by some miracle we got multiple glyphs so it can be used, this calculates which positions
        // glyphs are already placed in and how to place the current one(s) the robot possesses
        placeGlyph(CryptoKey);
    }
    public void findColumn(){
        //outdated:
        // Set the FoundPylon boolean to false, for the next part of the program in which we use
        // similar methodology as we have to far, but strafing instead of front-to-back motion
        // There's no tolerance code this time because we're pressed right up against the cryptobox
        // and since we have a flat back on our robot, we can just strafe from side to side and so
        // whenever a distance value is less than what the distance is to the wall, that means there's
        // a pylon in that location, and we can assume our position from there
        boolean FoundPylon = false;
        while(opModeIsActive() && !FoundPylon){
            if(CryptoboxDistance.getDistance(DistanceUnit.CM) < 7){
                FoundPylon = true;
            }else {
                moveBy(.025, -.5, 0); //moveBy is a function that handles robot movement
            }
        }
    }

    public void ramThePit(){
        EncoderDrive(.75, 24, 24, Forward);
        intakeGlyphs();
        findWall(-.25, 44);
        drive(0, .4, .25);
        findColumn();
        placeGlyph(CryptoKey);
        // if time < needed time go back
        // else pick up another
    }

    public void intakeGlyphs(){
        int GlyphsFound = 0;
        double startingEncoderCount = FrontLeft.getCurrentPosition();
        double limitEncoderCount = startingEncoderCount + 36*CountsPerInch;

        ConveyorLeft.setPower(1);
        ConveyorRight.setPower(1);
        TopIntakeServoLeft.setPower(1);
        TopIntakeServoRight.setPower(1);

        while(GlyphsFound < 2 && FrontLeft.getCurrentPosition() < limitEncoderCount && runtime.seconds() < 25){
            ConveyorLeft.setPower(1);
            ConveyorRight.setPower(1);
            TopIntakeServoLeft.setPower(1);
            TopIntakeServoRight.setPower(1);
            if(ConveyorDistance.getDistance(DistanceUnit.CM) < 30){
                //we have a glyph
                GlyphsFound += 1;;
            }else {
                moveBy(.1, 0, 0);
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
            }
        }
        stopDriveMotors();
        ConveyorLeft.setPower(0);
        ConveyorRight.setPower(0);
        TopIntakeServoLeft.setPower(0);
        TopIntakeServoRight.setPower(0);
    }
    // End movement methods
    // Motor and servo methods
    public void knockOffJewel(String AllianceColor){
        JewelArm.setPosition(JewelServoDownPos);
        //sleep(200);
        CryptoboxServo.setPosition(CryptoboxServoOutPos);
        telemetry.addData("KnockingJewel", 10);
        telemetry.update();
        // sleep to allow the jewel arm to go down
        sleep(800);
        // Decide which way to turn via a function
        double TurningAngle = 3*jewelDirection(AllianceColor);
        // Turn to knock off the jewel
        gyroTurn(.25, TurningAngle);
        JewelArm.setPosition(JewelServoUpPos);
        // Turn back to the original robot orientation
        gyroTurn(RegularTurnSpeed, 0);
        sleep(2000);
        }
    public int jewelDirection(String AllianceColor){
        int Blue = JewelColor.blue();
        int Red = JewelColor.red();
        int Direction = 0;
        if(Red > Blue){
            Direction = 1;
        }else{
            Direction = -1;
        }
        if (AllianceColor.equals("RED")){
            Direction = -Direction;
        }
        return Direction;
    }
    public void placeGlyph(RelicRecoveryVuMark Column){
        int BlockPos = 0;
         /*
         Since array goes from left-right, 0-3

         the array looks like |9|10|11|
         this when it's       |6| 7| 8|
         values are mapped    |3| 4| 5|
         to the cryptobox     |0| 1| 2|

         0 = open, 1 = grey, 2 = brown
         So a cypher may look |2| 1| 2|
         something like this  |1| 2| 1|
                              |2| 1| 2|
                              |1| 2| 1|

         0 = open, 1 = grey, 2 = brown
         An Empty  box  looks |0| 0| 0|
         something like this  |0| 0| 0|
                              |0| 0| 0|
                              |0| 0| 0|


         Get column
         figure out which number block(s) is dumping
         decide to either run conveyor or to dump
         */
        /*if(Column == RelicRecoveryVuMark.LEFT){
            BlockPos = 0;//First glyph location in this column
            while (CurrentCryptoBox[BlockPos] != 0 && opModeIsActive()) {
                BlockPos += 3;
            }
            dump(BlockPos);
        }else if (Column == RelicRecoveryVuMark.CENTER){
            BlockPos = 1;//First glyph location in this column
            while (CurrentCryptoBox[BlockPos] != 0 && opModeIsActive()) {
                BlockPos += 3;
            }
            dump(BlockPos);
        }else if (Column == RelicRecoveryVuMark.RIGHT){
            BlockPos = 2;//First glyph location in this column
            while (CurrentCryptoBox[BlockPos] != 0 && opModeIsActive()) {
                BlockPos += 3;
            }
            dump(BlockPos);
        }else{
            telemetry.addData("No CryptoKey, zilch, nada", Column);
            telemetry.update();
        }*/
        telemetry.addData("In conveyoring", 1);
        telemetry.update();
        DumpConveyor.setPower(1);
        Blocker.setPosition(BlockerServoDown);
        sleep(2000);
        EncoderDrive(.025, 2, 2
                , Forward);
        CryptoboxServo.setPosition(CryptoboxServoMidPos);
        EncoderDrive(.025, 10, 10, Reverse);
        EncoderDrive(.025, 3, 3, Forward);
        DumpConveyor.setPower(0);
        telemetry.addData("In end conveyor", 1);
        telemetry.update();
    }
    public void dump(int BlockPosition){
        telemetry.addData("In conveyoring", 1);
        telemetry.update();
        DumpConveyor.setPower(1);
        Blocker.setPosition(BlockerServoDown);
        sleep(2000);
        EncoderDrive(.025, 5, 5, Forward);
        EncoderDrive(.025, 10, 10, Reverse);
        EncoderDrive(.025, 3, 3, Forward);
        DumpConveyor.setPower(0);
        telemetry.addData("In end conveyor", 1);
        telemetry.update();
    }

    public double deriv(int counts) {
        // We messed around with PID in the offseason, but we decided there was no good application
        // for it in this years challenge.
        long elapsed = (currentTimeMillis() - PreviousTime)/1000;

        if (count_list[loop_counter] !=0 && time_list[loop_counter] != 0){
            denom = currentTimeMillis() - time_list[0];
            if (denom < .00001) {
                return previous_power;
            }
            num = counts - count_list[0];
            d = num / denom;

            for (int i = 0; i < time_list.length - 1; i++) {
                time_list[i] = time_list[i + 1];
            }
            for (int i = 0; i < count_list.length - 1; i++) {
                count_list[i] = count_list[i + 1];
            }
            time_list[time_list.length - 1] =  currentTimeMillis();
            count_list[count_list.length - 1] = counts;
        }

        if (time_list[loop_counter] == 0){
            time_list[loop_counter] = currentTimeMillis();
        }
        if(count_list[loop_counter] == 0){
            count_list[loop_counter] = counts;
        }
        if(loop_counter < (count_list.length - 1)){
            loop_counter += 1;
        }
        PreviousTime = currentTimeMillis();
        return d;
    }
    public void EncoderDriveWAccecAndDecel(double speed, double leftInches, double rightInches, double accelerationInches, double decelerationInches, int direction) {
        // Declares variables that are used for this method
        int NewLeftTarget;
        int NewRightTarget;
        int FrontLeftPosition;
        int FrontRightPosition;
        int BackLeftPosition;
        int BackRightPosition;

        double DecelTicks;
        double Speed = speed;
        double LeftSpeed;
        double RightSpeed;
        double DistanceBeforeDeceleration;
        double DecelTickMultiplier;
        double DecelSpeedVar;
        double MinSpeed = .15;
        double SpeedToDecelerate;

        boolean Running = true;
        telemetry.addData("Resetting Encoders", 1);
        telemetry.update();
        // Resets encoders to 0
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        if (Running) {
            // Determine new target position, and pass to motor controller
            // Calculates the needed encoder ticks by multiplying a pre-determined amount of CountsPerInches,
            // and the method input gets the actual distance travel in inches
            NewLeftTarget = FrontRight.getCurrentPosition() + (int) (leftInches * CountsPerInch);
            NewRightTarget = FrontRight.getCurrentPosition() + (int) (rightInches * CountsPerInch);
            // Gets the current position of the encoders at the beginning of the EncoderDrive method
            FrontLeftPosition = FrontLeft.getCurrentPosition();
            FrontRightPosition = FrontRight.getCurrentPosition();
            BackLeftPosition = BackLeft.getCurrentPosition();
            BackRightPosition = BackRight.getCurrentPosition();


            // Setup for deceleration
            telemetry.addData("Setup vals for decel", 1);
            telemetry.update();
            DecelTicks = ((decelerationInches * CountsPerInch));
            SpeedToDecelerate = speed - MinSpeed;
            DistanceBeforeDeceleration = Math.abs(NewLeftTarget) - Math.abs(DecelTicks);
            DecelTickMultiplier = (SpeedToDecelerate/DecelTicks);

            // Gives the encoders the target.
            telemetry.addData("Give targets to motors", 1);
            telemetry.update();
            FrontLeft.setTargetPosition(NewLeftTarget);
            FrontRight.setTargetPosition(NewRightTarget);
            BackLeft.setTargetPosition(NewLeftTarget);
            BackRight.setTargetPosition(NewRightTarget);


            /*// Turn On RUN_TO_POSITION
            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
*/
            // reset the timeout time and start motion.
            runtime.reset();
            // This gets where the motor encoders will be at full position when it will be at full speed.
            double LeftEncoderPositionAtFullSpeed = ((accelerationInches*(CountsPerInch)) + FrontLeftPosition);
            double RightEncoderPositionAtFullSpeed = ((accelerationInches*(CountsPerInch)) + FrontRightPosition);

            // This gets the absolute value of the encoder positions at full speed - the current speed, and while it's greater than 0, it will continues increasing the speed.
            // This allows the robot to accelerate over a set number of inches, which reduces wheel slippage and increases overall reliability
            while (opModeIsActive() && Running) {// && opModeIsActive
                // While encoders are not at position
                if ((((Math.abs(speed)) - (Math.abs(FrontLeft.getPower()))) > .05) && (
                        Math.abs(NewLeftTarget) - Math.abs(FrontLeft.getCurrentPosition()) > -1)
                        && (Math.abs(LeftEncoderPositionAtFullSpeed)- Math.abs(FrontLeft.getCurrentPosition()) > 10)){

                    LeftSpeed = (Range.clip(Math.abs((Math.abs(FrontLeft.getCurrentPosition())) / (Math.abs(LeftEncoderPositionAtFullSpeed))), MinSpeed, speed));
                    RightSpeed = (Range.clip(Math.abs((Math.abs(FrontRight.getCurrentPosition())) / (Math.abs(RightEncoderPositionAtFullSpeed))), MinSpeed, speed));

                    // This allows the robot to accelerate over a set distance, rather than going full speed.  This reduces wheel slippage and increases reliability.
                }else if(((Math.abs(NewRightTarget)-Math.abs(Math.abs(FrontRight.getCurrentPosition())) <= Math.abs(DecelTicks)))
                        && (Math.abs(NewLeftTarget) - Math.abs(FrontLeft.getCurrentPosition()) > -1)){
                    // Ramp down the power
                    DecelSpeedVar = ((Math.abs(FrontLeft.getCurrentPosition())-Math.abs(DistanceBeforeDeceleration)));
                    double DecelClipVar = DecelSpeedVar* Math.abs(DecelTickMultiplier);
                    LeftSpeed = Range.clip(((Math.abs(speed) - Math.abs(DecelClipVar))), MinSpeed, speed);
                    RightSpeed = Range.clip(((Math.abs(speed) - Math.abs(DecelClipVar))), MinSpeed, speed);
                    telemetry.addData("Decelerating", LeftSpeed);
                    telemetry.update();
                }else{
                    RightSpeed = speed;
                    LeftSpeed = speed;
                    telemetry.addData("Normal Speed", LeftSpeed);
                    telemetry.addData("Distance/Inches", FrontLeft.getCurrentPosition()/CountsPerInch);
                    telemetry.update();
                }
                if(Math.abs(NewLeftTarget) - Math.abs(FrontLeft.getCurrentPosition()) < -1 || !FrontLeft.isBusy()) {
                    //If absolute value of wanted encoder count at finish -
                    // the absolute value of current motor is < -1, then stop running.
                    Running = false;
                }
                FrontLeft.setPower(Range.clip(Math.abs(LeftSpeed), MinSpeed*direction,direction*speed));
                FrontRight.setPower(Range.clip(Math.abs(RightSpeed), MinSpeed*direction,direction*speed));
                BackLeft.setPower(Range.clip(Math.abs(LeftSpeed), MinSpeed*direction,direction*speed));
                BackRight.setPower(Range.clip(Math.abs(RightSpeed), MinSpeed*direction,direction*speed));
                telemetry.addData("Running", 10);
                telemetry.update();
            }

            // Stops all motion
            // Set to run without encoder, so it's not necessary to declare this every time after the method is used
            FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // Set power to 0
            telemetry.addData("Set power    ", 1);
            telemetry.update();
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);

        }
    }
}

