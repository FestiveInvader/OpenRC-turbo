package org.firstinspires.ftc.teamcode.Autonomous;

import android.hardware.Sensor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.openftc.hardware.rev.OpenRevDcMotorImplEx;

import java.util.Arrays;

//@Author Eric Adams, Team 8417 'Lectric Legends

public class DeclarationsAutonomous extends LinearOpMode {
    // This section declares hardware for the program, such as Motors, servos and sensors
    // Declare Motors
    public DcMotor FrontLeft = null;              // NeveRest Orbital 20
    public DcMotor BackLeft = null;               // NeveRest Orbital 20
    public DcMotor FrontRight = null;             // NeveRest Orbital 20
    public DcMotor BackRight = null;              // NeveRest Orbital 20
    public OpenRevDcMotorImplEx ConveyorLeft = null;           // Rev HD Hex Motor
    public OpenRevDcMotorImplEx ConveyorRight = null;          // Rev HD Hex Motor
    public DcMotor DumpingMotor = null;           // NeveRest 60
    public DcMotor LinearSlideMotor = null;       // NeveRest 20

    // Declare Servos
    public Servo JewelArm = null;                 // Rev SRS
    public Servo CryptoboxServo = null;           // Rev SRS
    public CRServo IntakeServo;
    public Servo FlipperServo;


    public DistanceSensor CryptoboxDistance;
    public ColorSensor JewelColor;
    public DigitalChannel DumperTouchSensorRight;
    public DigitalChannel DumperTouchSensorLeft;
    public DistanceSensor FlipperDistance2;
    public DistanceSensor RevBackDistance;
    public BNO055IMU IMU;
    public I2CXLv2 BackDistance;

    // Variables used  in functions
    double CountsPerRev = 537.6;    // Andymark NeveRest 20 encoder counts per revolution
    double GearRatio = 1;
    double WheelDiameterInches = 4.0;     // For figuring circumference
    double CountsPerInch = ((CountsPerRev / ((WheelDiameterInches * 3.1415))/GearRatio));
    double HEADING_THRESHOLD = 2;      // As tight as we can make it with an integer gyro
    double P_TURN_COEFF = .2;     // Larger is more responsive, but also less stable
    double P_DRIVE_COEFF = .15;     // Larger is more responsive, but also less stable
    double turningSpeed = .175;
    //empty is 0, grey is 1, brown is 2,
    int[] CurrentLeft = new int[]{0,0,0,0};
    int[] CurrentCenter = new int[]{0,0,0,0};
    int[] CurrentRight = new int[]{0,0,0,0};
    int[] FrogCypherBrownMid = new int[]{1,2,1,2,1,2,1,2,1,2,1,2};
    int[] FrogCypherGreyMid = new int[]{2,1,2,1,2,1,2,1,2,1,2,1};
    double cryptoboxHeading = 0;
    int Forward = 1;
    int Reverse = -1;
    int RelicSide = 1;
    int TeamSide = 2;

    int DumpingGearDriven = 40; // Gear connected to dumping motor
    int DumpingGearDriving = 80; // Gear connected to dumping assembly
    int DumpingDegreesOfTravel = 105; // Wanted degrees of the dump to travel
    int FractionOfRevolutionToDump = 360/DumpingDegreesOfTravel;
    int DumpingMotorEncoderTicks = 1680; // NeveRest 60
    int DumpingGearRatio = DumpingGearDriving/DumpingGearDriven; // 2:1
    int DumpingEncoderTicksPerRevolution = DumpingMotorEncoderTicks*DumpingGearRatio;
    int EncoderTicksToDump = DumpingEncoderTicksPerRevolution/FractionOfRevolutionToDump;
    int glyphs = 0;
    int trips = 0;


    boolean sensorsSeeTwo = false;
    boolean haveGlyphs = false;
    boolean FlipperServoUp = false;
    boolean eitherArePressed = false;
    double glyphsSeenTime;


    double BlockerServoUp = .3;
    double BlockerServoDown = .56;
    double FlipperServoUpPos = .2;
    double FlipperServoDownPos = 1;
    double IntakeServoUp = 1;
    double IntakeServo90Pos = .7;
    double IntakeServoDown = 0;

    double JewelServoUpPos = .575 ;
    double JewelServoDistancePos = .35;
    double JewelServoDownPos = .14; //.2 really
    double RegularTurnSpeed = .165;
    double IntakeSpeed = -.7;
    double CryptoboxServoInPos = 0;
    double CryptoboxServoOutPos = .85;
    double CryptoboxServoMidPos = .7;
    double programStartOrientation;
    double stayOnHeading = 84.17;

    int currentColumn = 0;
    boolean LeftColumnPlaced = false;
    boolean CenterColumnPlaced = false;
    boolean RightColumnPlaced = false;


    boolean knockedCryptoboxSideJewel = false;
    boolean haveGlyph = false;


    VuforiaLocalizer vuforia;
    VuforiaHardware vuforiaHardware;
    RelicRecoveryVuMark CryptoKey;
    RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
    public ElapsedTime runtime = new ElapsedTime();

    float d = 0;
    float denom = 0;
    float num = 0;
    int loop_counter = 0;
    int previous_power = 0;
    int count_list[]=new int[1];
    long time_list[]=new long[1];
    boolean columnsPlaced[] = new boolean[3];



    long PreviousTime = 0;
    int color = 0;
    @Override
    public void runOpMode() {
        // This section gets the hardware maps
        Arrays.fill(columnsPlaced, Boolean.FALSE);
        //Makes sure that the booleans start as false
        FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        FrontRight = hardwareMap.dcMotor.get("FrontRight");
        BackLeft = hardwareMap.dcMotor.get("BackLeft");
        BackRight = hardwareMap.dcMotor.get("BackRight");
        ConveyorLeft = new OpenRevDcMotorImplEx((DcMotorImplEx) hardwareMap.dcMotor.get("ConveyorLeft"));
        ConveyorRight = new OpenRevDcMotorImplEx((DcMotorImplEx) hardwareMap.dcMotor.get("ConveyorRight"));
        DumpingMotor = hardwareMap.dcMotor.get("DumpingMotor");
        LinearSlideMotor = hardwareMap.dcMotor.get("LinearSlideMotor");

        DumpingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ConveyorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        ConveyorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LinearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Hardware maps for servos
        JewelArm = hardwareMap.servo.get("JewelServo");
        CryptoboxServo = hardwareMap.servo.get("CryptoboxServo");
        FlipperServo = hardwareMap.servo.get("FlipperServo");

        // Initialize and hardware map Sensors
        DumperTouchSensorRight = hardwareMap.get(DigitalChannel.class, "DumperTouchSensorRight");
        DumperTouchSensorLeft = hardwareMap.get(DigitalChannel.class, "DumperTouchSensorLeft");
        DumperTouchSensorRight.setMode(DigitalChannel.Mode.INPUT);
        DumperTouchSensorLeft.setMode(DigitalChannel.Mode.INPUT);
        CryptoboxDistance = hardwareMap.get(DistanceSensor.class, "CryptoboxSensor");
        BackDistance = hardwareMap.get(I2CXLv2.class, "BackDistance");
        JewelColor = hardwareMap.get(ColorSensor.class, "JewelSensor");
        RevBackDistance = hardwareMap.get(DistanceSensor.class, "RevBackDistance");

        FlipperDistance2 = hardwareMap.get(DistanceSensor.class, "FlipperSensor2");

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
        vuforiaHardware = new VuforiaHardware();

        vuforiaHardware.Init(hardwareMap);
        JewelArm.setPosition(JewelServoUpPos);

        ElapsedTime timer = new ElapsedTime();
        while(timer.seconds() < 2){
            vuMark = vuforiaHardware.getVuMark();
            CryptoKey = vuMark;
            telemetry.addData("Vumark First Scan", CryptoKey);
            //telemetry.addData("Timer", timer.seconds());
            telemetry.update();
        }
        while((CryptoKey == vuMark || CryptoKey == RelicRecoveryVuMark.UNKNOWN) && !isStarted() ){
            CryptoKey = vuforiaHardware.getVuMark();
            telemetry.addData("Vumark Scanning", CryptoKey);
            telemetry.addData("If vumark same as last match ", vuforiaHardware.getVuMark());
            telemetry.addData("You can go ahead and run now ", vuforiaHardware.getVuMark());
            telemetry.update();
        }
        LinearSlideMotor.setPower(0);
        vuforiaHardware.Stop();
        while(!isStarted()) {
            telemetry.addData("Vumark FOUND!!!!", CryptoKey);
            telemetry.addData("IMU", getHeading());
            telemetry.addData("Back Distance", BackDistance.getDistance());
            telemetry.addData("FrontLeft Encoder", FrontLeft.getCurrentPosition());
            telemetry.addData("If values are good, then run", 1);
            telemetry.update();
        }

    }

    // Start Movement methods
    public void drive(double speed, int direction, double time){
        double startingHeading = getHeading();
        double timeStarted = runtime.time();
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(opModeIsActive() && runtime.time() - timeStarted < time && runtime.seconds() < 29.75) {
            gyroDrive(startingHeading, speed, direction);
        }
        stopDriveMotors();
    }
    public void driveWStrafe(double yspeed, double xspeed, double rotation, double time){
        double startingHeading = getHeading();
        double timeStarted = runtime.seconds();
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(opModeIsActive() && runtime.seconds() - timeStarted < time) {
            moveBy(yspeed, xspeed, rotation);
        }
        stopDriveMotors();
    }

    public void EncoderDrive(double speed, double Inches, int direction, double heading, double timeout) {
        //Here's the encoder drive Michael
        double startTime = runtime.seconds();
        double Heading = 0;
        boolean notAtTarget = true;
        if (heading == 84.17){
            Heading = getHeading();
        }else{
            Heading = heading;
        }
        double target;
        if (opModeIsActive() ) {
            FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            target = FrontLeft.getCurrentPosition() + (int) (Inches * CountsPerInch * direction);
            while(opModeIsActive() && notAtTarget && Math.abs(target) - Math.abs(FrontLeft.getCurrentPosition()) > 25
                    && (startTime + timeout > runtime.seconds())) {

                gyroDrive(Heading, speed, direction);
                //smartIntake(); Conveyor/intake

            }
            stopDriveMotors();
        }
    }
    public void EncoderDriveWSmartIntake(double speed, double Inches, int direction, double heading, double timeout) {
        //Here's the encoder drive Michael
        double startTime = runtime.seconds();
        double Heading = 0;
        boolean notAtTarget = true;
        double target;
        if (opModeIsActive() ) {
            FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            target = FrontLeft.getCurrentPosition() + (int) (Inches * CountsPerInch * direction);
            while(opModeIsActive() && notAtTarget && Math.abs(target) - Math.abs(FrontLeft.getCurrentPosition()) > 25
                    && (startTime + timeout > runtime.seconds())) {
                moveBy(speed, 0, .2 * -heading);
                smartIntake();
                //smartIntake(); Conveyor/intake

            }
            stopDriveMotors();
        }
    }
    public void EncoderDriveAccelDecel(double speed, double inches, double decelInches, int direction, double heading, double timeout){
        double startTime = runtime.seconds();
        double Heading = 0;
        boolean notAtTarget = true;
        if (heading == 84.17){
            Heading = getHeading();
        }else{
            Heading = heading;
        }
        double target;
        if (opModeIsActive() ) {
            FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



            FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double power = 0;
            double error;


            target = FrontLeft.getCurrentPosition() + (int) (inches * CountsPerInch * direction);
            double decelTicks = (int) (decelInches * CountsPerInch);

            while(Math.abs(target) - Math.abs(FrontLeft.getCurrentPosition()) > 25 && runtime.seconds() < 28.5 && (startTime + timeout > runtime.seconds())) {
                double motorPos = Math.abs(FrontLeft.getCurrentPosition());
                error = Math.abs(target) - Math.abs(motorPos);
                //decel
                if(Math.abs(motorPos) < Math.abs(target) - Math.abs(decelTicks)){
                    gyroDrive(Heading, speed, direction);
                    telemetry.addData("FrontLeftPwr", FrontLeft.getPower());
                    telemetry.addData("In speeeeeeeed", FrontLeft.getPower());
                    telemetry.update();
                }else {
                    gyroDrive(Heading, Range.clip(power, .15, 1), direction);
                    telemetry.addData("FrontLeftPwr", FrontLeft.getPower());
                    telemetry.addData("In Decel", FrontLeft.getPower());
                    telemetry.update();
                }

            }
            stopDriveMotors();
        }
    }

    public void gyroTurn(double speed, double angle) {
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, -angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.addData("Target Rot", angle);
            telemetry.addData("Current Rot", getHeading());
            telemetry.update();
        }
    }
    public void gyroTurnNotNegative ( double speed, double angle) {
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
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
        double minTurnSpeed = .15;
        double maxTurnSpeed = .5;
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
            leftSpeed  = Range.clip(speed + Math.abs(error/175) , minTurnSpeed, maxTurnSpeed)* PosOrNeg;

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
    public void gyroDrive(double targetAngle, double targetSpeed, int direction) {
        //Here's the drive code Michael
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double LeftPower = 0;
        double RightPower = 0;
        int Direction = -direction;
        double diff = -getError(targetAngle);
        double PVal = 15/targetSpeed; // Play with this value
        if(Direction == -1){
            LeftPower = Direction*(targetSpeed+diff/PVal);
            RightPower = Direction*(targetSpeed-diff/PVal);
        }else{
            LeftPower = Direction*(targetSpeed-diff/PVal);
            RightPower = Direction*(targetSpeed+diff/PVal);
        }

        FrontLeft.setPower(Range.clip(LeftPower, -1, 1));
        FrontRight.setPower(Range.clip(RightPower, -1, 1));
        BackLeft.setPower(Range.clip(LeftPower, -1, 1));
        BackRight.setPower(Range.clip(RightPower, -1, 1));
    } //driveAdjust
    public double getHeading(){
        Orientation angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
    public void moveBy(double y, double X, double c) {
        double x = -X;
        double FrontLeftVal = -y + x + c;
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
    public void moveByWithRotation(double y, double x, double C) {
        double c = getError(C);
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
    public void stopDriveMotors(){
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
    }

    //This year's specific movement and logic functions
    public void driveToCrypotboxEncoders(int Direction, int startingPosition) {
        int PylonsToFind = cryptoboxPylonsToGo(Direction);
        double DistanceToTravel = 0;
        // Blue side
        if(startingPosition == 1 || startingPosition == 4){
            if(Direction == Reverse){
                if (PylonsToFind == 0) {
                    DistanceToTravel = 2;
                    //4?
                } else if (PylonsToFind == 1) {
                    DistanceToTravel = 10;

                } else if (PylonsToFind == 2) {
                    //18?
                    DistanceToTravel = 18;
                }
            }else{
                if (PylonsToFind == 0) {
                    DistanceToTravel = 5;
                    //4?
                } else if (PylonsToFind == 1) {
                    DistanceToTravel = 13;

                } else if (PylonsToFind == 2) {
                    //18?
                    DistanceToTravel = 21;
                }
            }
        }else{
            if(Direction == Forward){
                //red
                if (PylonsToFind == 0) {
                    DistanceToTravel = 2;
                } else if (PylonsToFind == 1) {
                    DistanceToTravel = 6.5;
                } else if (PylonsToFind == 2) {
                    DistanceToTravel = 14;
                }
            }else{
                if (PylonsToFind == 0) {
                    DistanceToTravel = 0;
                    //4?
                } else if (PylonsToFind == 1) {
                    DistanceToTravel = 8;

                } else if (PylonsToFind == 2) {
                    //18?
                    DistanceToTravel = 16;
                }
            }
        }

        if(startingPosition == 1 || startingPosition == 4) {
            if (Direction == Reverse) {
                if (knockedCryptoboxSideJewel) {
                    EncoderDriveAccelDecel(.35, 11 + DistanceToTravel, 12, Direction, stayOnHeading, 1.75);
                } else {
                    EncoderDriveAccelDecel(.35, 15 + DistanceToTravel, 12, Direction,stayOnHeading, 1.75);
                }
            } else {
                if (knockedCryptoboxSideJewel) {
                    EncoderDrive(.35, 12 + DistanceToTravel, Direction, stayOnHeading, 1.75);
                } else {
                    EncoderDrive(.35, 16 + DistanceToTravel, Direction, stayOnHeading, 1.75);
                }
            }
        }else{
            EncoderDrive(.25,  DistanceToTravel,  Forward, stayOnHeading, 1.5);
        }
        if(Direction == Reverse) {
            //blue side
            if (PylonsToFind == 0) {
                columnsPlaced[0] = true;
            } else if (PylonsToFind == 1) {
                columnsPlaced[1] = true;
            } else {
                columnsPlaced[2] = true;
            }
        }else{
            //red side
            if (PylonsToFind == 0) {
                columnsPlaced[2] = true;
            } else if (PylonsToFind == 1) {
                columnsPlaced[1] = true;
            } else {
                columnsPlaced[0] = true;
            }
        }
        currentColumn = PylonsToFind;
    }
    public int cryptoboxPylonsToGo(int Direction){
        int PylonsToFind = 0;
        // This allows us to use one function for all autonomous programs, since this covers all
        // cases of use.
        // If we travel forward to get to the cryptobox.  Since the distance sensor is on
        // the right, this means we can set values based on which direction we're going
        if (CryptoKey.equals(RelicRecoveryVuMark.LEFT) && Direction == Forward) {
            PylonsToFind = 2;
        } else if (CryptoKey.equals(RelicRecoveryVuMark.CENTER) && Direction == Forward) {
            PylonsToFind = 1;
        } else if (CryptoKey.equals(RelicRecoveryVuMark.RIGHT) && Direction == Forward) {
            PylonsToFind = 0;
        } else if (CryptoKey.equals(RelicRecoveryVuMark.UNKNOWN) && Direction == Forward) {
            // No target, Vumark failed recognition.  Put in Near column
            PylonsToFind = 0;
        } else if (CryptoKey.equals(RelicRecoveryVuMark.LEFT) && Direction == Reverse) {
            // Else, we're traveling backwards, which means we are on the blue alliance
            PylonsToFind = 0;
        } else if (CryptoKey.equals(RelicRecoveryVuMark.CENTER) && Direction == Reverse) {
            PylonsToFind = 1;
        } else if (CryptoKey.equals(RelicRecoveryVuMark.RIGHT) && Direction == Reverse) {
            PylonsToFind = 2;
        } else if (CryptoKey.equals(RelicRecoveryVuMark.UNKNOWN) && Direction == Reverse) {
            // No target, Vumark failed recognition.  Put in Near column
            PylonsToFind = 0;
        }
        return PylonsToFind;
    }

    public void turnToCryptobox (int startingPosition){
        int heading = 0;
        if(startingPosition == 1){
            heading = -87;
        }
        if(startingPosition == 2){
            heading = -3;
        }
        if(startingPosition == 3){
            heading = -181;
        }
        if(startingPosition == 4){
            heading = -92;
        }
        gyroTurn(turningSpeed, heading);
    }

    public void extendCryptoboxArmForFirstGlyph(){
        CryptoboxServo.setPosition(CryptoboxServoOutPos);
        EncoderDrive(.175, 6.5, Forward, stayOnHeading, 5);
        EncoderDrive(.2, 3.75, Reverse, stayOnHeading, 2);
    }

    public void driveAndPlace(RelicRecoveryVuMark CryptoKey, int Direction, int Placement, double gyroOffset, int startingPosition){
        // Tolerance +- of the beginning distance, to account for small mistakes when setting robot up
        // and while knocking the jewel off
        if(startingPosition == 2 || startingPosition == 3){
            if(Direction == Forward){
                // Red side, far stone
                goToDistance(.2, 40, BackDistance, 1.75, 2);
            }else{
                //blue side, far stone
                goToDistance(.2, 38 , BackDistance, 1.65, 2);
            }
        }
        driveToCrypotboxEncoders(Direction, startingPosition);

        stopDriveMotors();
        turnToCryptobox(startingPosition);
        findWallRevSensor(-.3, 100,1.5);
        extendCryptoboxArmForFirstGlyph();
        findColumn(2.25);
        stopDriveMotors();
        placeByFlippingFirstGlyph(3);


    }
    public void endAuto(){
        JewelArm.setPosition(JewelServoDistancePos);
        EncoderDrive(.35, 6, Forward, stayOnHeading, 2);
        CryptoboxServo.setPosition(CryptoboxServoInPos);
        drive(.2, Forward, .5);
        drive(.2, Reverse, .75);
        drive(.2, Forward, .5);
        JewelArm.setPosition(JewelServoUpPos);
        telemetry.addData("Vumark", CryptoKey);
        telemetry.addData("Color", color);
        telemetry.addData("No Glyphs", glyphs);
        telemetry.addData("KnockedCryptoboxJewl", knockedCryptoboxSideJewel);
        telemetry.update();
        sleep(2500);

    }
    public void findColumn(double timeout){
        double startTime = runtime.seconds();
        double Timeout = startTime + timeout;
        //outdated:
        // Set the FoundPylon boolean to false, for the next part of the program in which we use
        // similar methodology as we have to far, but strafing instead of front-to-back motion
        // There's no tolerance code this time because we're pressed right up against the cryptobox
        // and since we have a flat back on our robot, we can just strafe from side to side and so
        // whenever a distance value is less than what the distance is to the wall, that means there's
        // a pylon in that location, and we can a ssume our position from there
        boolean FoundPylon = false;
        while(opModeIsActive() && !FoundPylon && Timeout - runtime.seconds() > .1){
            if (CryptoboxDistance.getDistance(DistanceUnit.CM) < 6.5 ) {
                moveBy(.015, .325, 0); //moveBy is a function that handles robot movement
            }else if(CryptoboxDistance.getDistance(DistanceUnit.CM) < 7.25){
                FoundPylon = true;
            }else {
                moveBy(.015, -.325, 0); //moveBy is a function that handles robot movement
            }
        }
        /*double error;
        double StrafingPSpeed = 0;
        double targetVal = 6.5;
        double PVal = 10;
        while(opModeIsActive() && !FoundPylon && Timeout - runtime.seconds() > .1){
            double currentVal = CryptoboxDistance.getDistance(DistanceUnit.CM);


            if(CryptoboxDistance.getDistance(DistanceUnit.CM)< 100) {
                error = (targetVal - currentVal) / PVal;
                StrafingPSpeed = Range.clip(error, .25, .4);
                moveBy(.015, StrafingPSpeed, 0);
                if(Math.abs(targetVal - currentVal) < .5){
                    FoundPylon = true;
                }
            }else{
                moveBy(.015, -.3, 0); //moveBy is a function that handles robot movement
            }

        }*/

        //
        stopDriveMotors();
    }
    public void findWall(double speed, double distance, double Timeout){
        int badLoopTimer = 0;
        double startHeading = getHeading();
        boolean foundWall = false;
        double timeout = runtime.seconds() + Timeout;
        int ThisLoopDistance;
        while (opModeIsActive() && !foundWall && (runtime.seconds() < timeout)) {
            ThisLoopDistance = BackDistance.getDistance();
            if(ThisLoopDistance > 200 || ThisLoopDistance < 21){
                //sensor val is bad, skip this loop
                moveBy(speed, 0, 0);
                badLoopTimer++;
            }else if(ThisLoopDistance > distance){
                moveBy(speed, 0, 0);
            }else{
                stopDriveMotors();
                foundWall = true;
            }
            telemetry.addData("Distance", BackDistance.getDistance());
            telemetry.update();
            smartIntake();
        }
    }
    public void findWallRevSensor(double speed, double distance, double Timeout){
        int badLoopTimer = 0;
        double startHeading = getHeading();
        boolean foundWall = false;
        double timeout = runtime.seconds() + Timeout;
        while (opModeIsActive() && !foundWall && (runtime.seconds() < timeout)) {
            if(RevBackDistance.getDistance(DistanceUnit.CM) < distance){
                stopDriveMotors();
                foundWall = true;
            }else{
                moveBy(speed, 0, 0);
            }
            telemetry.addData("Distance", BackDistance.getDistance());
            telemetry.update();
            smartIntake();
        }
    }

    public void goToDistance(double targetSpeed, double distance,  I2CXLv2 distanceSensor, double timeout, int tolerance){
        double startTime = runtime.seconds();
        double maxTime = startTime + timeout;
        double startHeading = getHeading();
        boolean foundTarget = false;
        int ThisLoopDistance;
        while (opModeIsActive() && !foundTarget && maxTime - runtime.seconds() > .1) {
            ThisLoopDistance = BackDistance.getDistance();
            double error = distance - ThisLoopDistance;
            int Direction = (int) Range.clip(error, -1, 1);

            if(ThisLoopDistance > 500 || ThisLoopDistance < 21){
                gyroDrive(startHeading, Range.clip(Math.abs(error/70), .135, targetSpeed), Direction);
                //sensor val is bad, stop bot so it doesn't go too far
            }else if(ThisLoopDistance > distance + tolerance || ThisLoopDistance < distance - tolerance){
                gyroDrive(startHeading, Range.clip(Math.abs(error/70), .135, targetSpeed), Direction);
            }else{
                stopDriveMotors();
                foundTarget = true;
            }
            telemetry.addData("Distance", ThisLoopDistance);
            telemetry.addData("error", error);
            telemetry.addData("speed", FrontLeft.getPower());
            telemetry.update();
        }
        stopDriveMotors();
    }

    public int getColumnsToTravelMG(){
        telemetry.addData("started get column", 1);
        telemetry.update();
        int columnToGoTo = 0;
        int columnsToTravel = 2;
        boolean foundColumnToGoTo = false;
        int c = 0;
        /*for(int i = 0; !columnsPlaced[i]; i++){
            columnToGoTo = i;
        }*/
        if(trips == 0){
            columnToGoTo = currentColumn;
        }else {
            if (!columnsPlaced[0]) {
                columnToGoTo = 0;
            } else if (!columnsPlaced[1]) {
                columnToGoTo = 1;
            } else if (!columnsPlaced[2]) {
                columnToGoTo = 2;
            } else {
                //nowhere left to place
            }
        }
        /*for(boolean e : columnsPlaced) {
            if(e) {
                columnToGoTo = c;
                telemetry.addData("Column that's free", columnToGoTo);
                telemetry.update();
            }
            c++;
        }*/
        columnsToTravel = currentColumn - columnToGoTo;
        telemetry.addData("Columns to go", columnsToTravel);
        telemetry.update();
        currentColumn = columnToGoTo;
        return columnsToTravel;
    }
    public void strafeToColumnMG(int direction){
        int columnsToGo = getColumnsToTravelMG();
        boolean strafingLeft = false;
        int strafingDirectionMultiplier = 1;
        if(columnsToGo > 0){
            strafingDirectionMultiplier = 1;
        }else{
            strafingDirectionMultiplier = -1;
        }
        double strafeTime = 1;


        if(columnsToGo == 0){
            //do nothing, there's no places left to place.  Or maybe place in right column?, chances
            //are, we'll never get this far but it's possible
	    //I mean, we made it to this point, so maybe just end up placing the glyphs anyway?
        }else{
            if(columnsToGo > 0){
                strafingLeft = false;
            }else{
                strafingLeft = true;
            }
            //blue side
            if(strafingLeft){
                strafeTime = .8;
            }else {
                strafeTime = 1.2;
            }
        }
            //}
            //Put in other function, just to see if it's the movement that's handled by this function that's out of whack
            //Y speed, x speed, rotation speed, time
        driveWStrafe(0, .4*columnsToGo, 0, 1);

        telemetry.addData("Strafe Time", strafeTime);
        telemetry.addData("Strafing Left?", strafingLeft);
        telemetry.addData("Strafing Multiplier", strafingDirectionMultiplier);
        telemetry.addData("Columns To Go", columnsToGo);
        telemetry.addData("Columns placed", columnsPlaced);
        telemetry.update();
    }
    public void MGAutoRelicSide(int startingPosition, int direction, int noOfTrips){
        while (trips < noOfTrips){
            ramThePitRelicSide(startingPosition,direction);
            trips++;
            telemetry.addData("Trips:", trips);
            telemetry.update();
        }
        endAuto();
    }
    public void ramThePitRelicSide(int startingPosition, int direction) {
        double startingHeading = getHeading();
        //Drive forward to pit
        // get glyphs
        // drive back to close to cryptobox
        // go to correct column
        // ram wall
        // find column
        // place glyph
        //if(runtime.seconds() < 25) {
        //Drive forward to pit - Should be complete
        if(trips == 1){
            gyroTurn(turningSpeed, startingHeading);
        }
        FlipperServo.setPosition(FlipperServoDownPos);
        CryptoboxServo.setPosition(CryptoboxServoMidPos);
        ConveyorLeft.setPower(1);
        ConveyorRight.setPower(1);
        EncoderDrive(1, 30, Forward, stayOnHeading, .75);

        int turningDirection = 1;
        if(startingPosition == 1){
            turningDirection = 1;
        }else{
            turningDirection = -1;
        }
        //Grab glyphs, this part needs work (also hardware side tho)

        driveToGlyphs(0, 2.25, .225);
        double inchesToDrive = FrontLeft.getCurrentPosition()/CountsPerInch;
        EncoderDriveWSmartIntake(-.5, Math.abs(inchesToDrive), Reverse, 0, .75);
        if(!haveGlyph()){
            double rotationOfCryptobox = -getHeading();
            if(startingPosition == 1) {
                gyroTurn(turningSpeed, rotationOfCryptobox + 30);
            }else{
                gyroTurn(turningSpeed, rotationOfCryptobox - 30);
            }
            driveToGlyphs(turningDirection, 1.5,.3);
            inchesToDrive = FrontLeft.getCurrentPosition()/CountsPerInch;
            EncoderDriveWSmartIntake(-.5, Math.abs(inchesToDrive), Reverse, 0, .75);
        }

        CryptoboxServo.setPosition(CryptoboxServoMidPos);
        turnToCryptobox(startingPosition);
        ConveyorRight.setPower(0);
        ConveyorLeft.setPower(0);
        FlipperServo.setPosition(FlipperServoUpPos);
        double rotationOfCryptobox = -getHeading();
        gyroTurn(turningSpeed, rotationOfCryptobox - 5);
        findWallRevSensor(-.3, 100, 4);
        FlipperServo.setPosition(FlipperServoDownPos);
        //go to correct column - need some work, just strafeToColumn reliability/distances/power values
        /*if(trips == 1){
            turnToCryptobox(startingPosition);
        }
        strafeToColumnMG(direction);*/

        //This section makes ure that we line up with the column we placed the first glyph in
        ConveyorRight.setPower(1);
        ConveyorLeft.setPower(1);
        //turnToCryptobox(startingPosition);
        findWallRevSensor(-.3, 100, 4);
        turnToCryptobox(startingPosition);

        FlipperServo.setPosition(FlipperServoUpPos);

        extendCryptoboxArmForFirstGlyph();
        EncoderDrive(.2, 3, Reverse, stayOnHeading, .5);
        //Find column and place glyphs - complete
        ConveyorLeft.setPower(-1);
        ConveyorRight.setPower(-1);
        findColumn(1.5);
        placeByFlippingSecondGlyph(2.25);
        //no more glyphs, end auton
        drive(.3, Reverse, 1.5);
    }

    public void ramThePitTeamSide(int startingPosition, int direction){
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double startingRotation = getHeading();
        EncoderDrive(.3, 4, Forward, stayOnHeading, .5);
        FlipperServo.setPosition(FlipperServoDownPos);
        double turningAngle = 0;
        if( startingPosition == 2){
            turningAngle = -90;
        }else{
            turningAngle =  -90;
        }
        gyroTurn(turningSpeed, turningAngle);
        int startingDistance = BackDistance.getDistance();
        goToDistance(.2, 70, BackDistance, 1, 2);
        if( startingPosition == 2){
            turningAngle = -35;
        }else{
            turningAngle =  -155;
        }
        gyroTurn(turningSpeed, turningAngle);
        ConveyorLeft.setPower(1);
        ConveyorRight.setPower(1);
        EncoderDrive(.95, 44, Forward, stayOnHeading, 2.5);
        sleep(500);


        driveToGlyphs(0, .75, .25);
       /* double inchesToDrive = FrontLeft.getCurrentPosition()/CountsPerInch;
        EncoderDriveWSmartIntake(-.5, Math.abs(inchesToDrive), Reverse, 0, .75);
*/
        CryptoboxServo.setPosition(CryptoboxServoMidPos);

        ConveyorRight.setPower(0);
        ConveyorLeft.setPower(0);
        FlipperServo.setPosition(FlipperServoUpPos);
        FlipperServo.setPosition(FlipperServoUpPos);


        /*findWall(-.95, 110, .75);
        if(startingPosition == 2){
            gyroTurn(turningSpeed, -90);
        }else{
            gyroTurn(turningSpeed, -90);
        }
        FlipperServo.setPosition(FlipperServoDownPos);
        if(startingPosition == 2) {
            goToDistance(.3, startingDistance - 3, BackDistance, 2, 2);
        }else{
            goToDistance(.3, startingDistance + 3, BackDistance, 2, 2);

        }*/

        //go to correct column - need some work, just strafeToColumn reliability/distances/power values
        /*if(trips == 1){
            turnToCryptobox(startingPosition);
        }
        strafeToColumnMG(direction);*/

        //This section makes ure that we line up with the column we placed the first glyph in
        ConveyorRight.setPower(1);
        ConveyorLeft.setPower(1);
        EncoderDrive(.95, 36, Reverse, stayOnHeading, 1 );
        FlipperServo.setPosition(FlipperServoUpPos);
        turnToCryptobox(startingPosition);
        findWallRevSensor(-.25, 100, 2.5);
        extendCryptoboxArmForFirstGlyph();
        EncoderDrive(.2, 3, Reverse, stayOnHeading, .5);
        //Find column and place glyphs - complete
        ConveyorLeft.setPower(-1);
        ConveyorRight.setPower(-1);
        findColumn(1.5);
        placeByFlippingSecondGlyph(2.25);
        //no more glyphs, end auton
        drive(.3, Reverse, 1.5);
        drive(.5, Forward, .75);
        drive(.3, Reverse, 1.5);

    }

    public void driveToGlyphs(int turningDirection, double timeout, double speed){
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double startingRotation = getHeading();
        double startingEncoderCount = FrontLeft.getCurrentPosition();
        double time = runtime.seconds() + timeout;
        //double limitEncoderCount = startingEncoderCount + inchesToGo*CountsPerInch;
        //&& Math.abs(limitEncoderCount) - Math.abs(FrontLeft.getCurrentPosition()) > 25
        while(!haveGlyph() && runtime.seconds() < 28  && time > runtime.seconds() && opModeIsActive() )  {
            gyroDrive(startingRotation, speed,Forward);
            smartIntake();
            telemetry.addData("Distance", FlipperDistance2.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        stopDriveMotors();
    }
    public boolean haveGlyph(){
        if (FlipperDistance2.getDistance(DistanceUnit.CM) <= 25) {
            haveGlyph = true;
        }else{
            haveGlyph = false;
        }
        return  haveGlyph;
    }
    public void intakeGlyph() {
        //Used with old bot
        boolean haveGlyph = false;
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double startingEncoderCount = FrontLeft.getCurrentPosition();
        double limitEncoderCount = startingEncoderCount + 8*CountsPerInch;
        ConveyorLeft.setPower(1);
        ConveyorRight.setPower(1);
        while(!haveGlyph && (Math.abs(FrontLeft.getCurrentPosition()) < Math.abs(limitEncoderCount)) && runtime.seconds() <= 25){
            if(FlipperDistance2.getDistance(DistanceUnit.CM) > 1){
                stopDriveMotors();
                haveGlyph = true;
            }else {
                moveBy(.135, 0, 0);
                //smartIntake(); Conveyor/intake
            }
        }

        stopDriveMotors();
        double inchesToDrive = FrontLeft.getCurrentPosition()/CountsPerInch;
        EncoderDrive(1, Math.abs(inchesToDrive), Reverse, stayOnHeading, 1.5);
    }
 // End movement methods
    // Motor and servo methods
    public void knockOffJewel(String AllianceColor, int startingPosition){
        JewelArm.setPosition(JewelServoDownPos);
        CryptoboxServo.setPosition(CryptoboxServoOutPos);
        knockedCryptoboxSideJewel = false;
        telemetry.addData("KnockingJewel", 10);
        telemetry.update();
        // sleep to allow the jewel arm to go down
        sleep(750);
        //Knock off the jewel.  If the jewel is the way we go to get the cryptobox, we drive forward
        // To knock off, otherwise we turn.  This is to
        int Direction = jewelDirection(AllianceColor);
        int turningAmount = 7;
        if(startingPosition == 1){
            if(Direction == Forward) {
                knockedCryptoboxSideJewel = true;
                EncoderDrive(.5, 5, Reverse, stayOnHeading, 1.25);
            }else{
                double TurningAngle = turningAmount * Direction;
                gyroTurn(turningSpeed, TurningAngle);
            }
        }else if (startingPosition == 4){
            if(Direction == Reverse) {
                knockedCryptoboxSideJewel = true;
                EncoderDrive(.5, 5, Forward, stayOnHeading, 1.25);
            }else{
                double TurningAngle = turningAmount * Direction;
                gyroTurn(turningSpeed, TurningAngle);
            }
        }else{
            double TurningAngle = turningAmount * Direction;
            gyroTurn(turningSpeed, TurningAngle);
        }
        JewelArm.setPosition(JewelServoUpPos);
        // Turn back to the original robot orientation
        gyroTurn(turningSpeed, 0);
        CryptoboxServo.setPosition(CryptoboxServoInPos);
    }
    public int jewelDirection(String AllianceColor){
        int Blue = JewelColor.blue();
        int Red = JewelColor.red();
        int Direction = 0;
        if(Red > Blue){
            //Jewel that sensor is pointing at is red, means that the robot will need to move
            Direction = -1;
        }else{
            //Jewel that sensor is pointing at is blue
            Direction = 1;
        }
        if (AllianceColor.equals("RED")){
            //if the alliance
            Direction = -Direction;
        }
        return Direction;
    }

    public void placeByFlippingFirstGlyph(double timeout){
        FlipperServo.setPosition(FlipperServoUpPos);
        double startTime = runtime.seconds();
        boolean placed = false;
        EncoderDrive(.35, 2, Forward, stayOnHeading, 2);
        double dumpingPower = .65;
        DumpingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DumpingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(!placed && startTime + 1 > runtime.seconds()) {
            DumpingMotor.setTargetPosition(DumpingMotor.getCurrentPosition() - EncoderTicksToDump);
            while (DumpingMotor.isBusy()) {
                DumpingMotor.setPower(-dumpingPower);
                telemetry.addData("Current Pos", DumpingMotor.getCurrentPosition());
                telemetry.addData("Target Pos", DumpingMotor.getTargetPosition());
                telemetry.addData("Difference for while", (Math.abs(DumpingMotor.getCurrentPosition())) - 1200);
                telemetry.update();
                //moveBy(.075, 0, 0);
            }
            placed = true;

        }
        sleep(150);
        FlipperServo.setPosition(FlipperServoDownPos);
        EncoderDrive(.15, 5, Forward, stayOnHeading, 1.5);
        DumpingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double startingRuntime = runtime.seconds();
        while ((DumperTouchSensorRight.getState() || DumperTouchSensorLeft.getState()) && runtime.seconds() - startingRuntime < .5 && opModeIsActive()) {
            DumpingMotor.setPower(dumpingPower);
            moveBy(-.3, 0,0);
        }
        DumpingMotor.setPower(0);
    }
    public void placeByFlippingSecondGlyph(double timeout){
        FlipperServo.setPosition(FlipperServoUpPos);
        double startTime = runtime.seconds();
        boolean placed = false;
        EncoderDrive(.35, 3, Forward, stayOnHeading, 2);
        CryptoboxServo.setPosition(CryptoboxServoMidPos);
        JewelArm.setPosition(JewelServoDistancePos);
        double dumpingPower = .65;
        DumpingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DumpingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(!placed && startTime + 1 > runtime.seconds()) {
            DumpingMotor.setTargetPosition(DumpingMotor.getCurrentPosition() - EncoderTicksToDump);
            while (DumpingMotor.isBusy()) {
                DumpingMotor.setPower(-dumpingPower);
                telemetry.addData("Current Pos", DumpingMotor.getCurrentPosition());
                telemetry.addData("Target Pos", DumpingMotor.getTargetPosition());
                telemetry.addData("Difference for while", (Math.abs(DumpingMotor.getCurrentPosition())) - 1200);
                telemetry.update();
            }
            placed = true;
            stopDriveMotors();
        }
        sleep(150);
        FlipperServo.setPosition(FlipperServoDownPos);
        drive(.25, Reverse, 1);
        EncoderDrive(.25, 1.5, Forward, stayOnHeading, 1.5);
        DumpingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double startingRuntime = runtime.seconds();
        CryptoboxServo.setPosition(CryptoboxServoInPos);
        while ((DumperTouchSensorRight.getState() || DumperTouchSensorLeft.getState()) && runtime.seconds() - startingRuntime < .5 && opModeIsActive()) {
            DumpingMotor.setPower(dumpingPower);
            moveBy(-.3, 0,0);
        }
        DumpingMotor.setPower(0);
        stopDriveMotors();
    }
    public void pushInFirstGlyph (){
        drive(.4, Reverse, .5);
    }
    public void smartIntake() {
         double intakeValLeft = 20;
        double speed = 1;

        /*if(ConveyorRight.getCurrentDraw() > 3500){
            ConveyorRight.setPower(-speed);
        }else if (ConveyorRight.getCurrentDraw() < 1250){
            ConveyorRight.setPower(speed);

        }
        if(ConveyorLeft.getCurrentDraw() > 3500){
            ConveyorLeft.setPower(-speed);
        }else if (ConveyorLeft.getCurrentDraw() < 1250){
            ConveyorLeft.setPower(speed);

        }*/
       if(ConveyorRight.getCurrentDraw() > 4500 || ConveyorLeft.getCurrentDraw() > 4500){
           ConveyorLeft.setPower(-1);
           ConveyorRight.setPower(-1);
       }else if((ConveyorRight.getCurrentDraw() < 2000 && ConveyorLeft.getCurrentDraw() < 2000)){
           ConveyorRight.setPower(speed);
           ConveyorLeft.setPower(speed);
       }
       sleep(20);
        telemetry.addData("Left Vel", ConveyorLeft.getCurrentDraw());
        telemetry.addData("Right Vel", ConveyorRight.getCurrentDraw());
        telemetry.update();
    }

}