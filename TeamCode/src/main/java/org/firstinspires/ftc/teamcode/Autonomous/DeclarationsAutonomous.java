package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
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

import java.util.Arrays;

//@Author Eric Adams, Team 8417 'Lectric Legends

public class DeclarationsAutonomous extends LinearOpMode {
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
    public Servo Blocker = null;                  // Rev SRS  Heh, block-er
    public Servo JewelArm = null;                 // Rev SRS
    public Servo CryptoboxServo = null;           // Rev SRS
    public Servo IntakeServo;
    public Servo FlipperServo;


    public DistanceSensor CryptoboxDistance;
    public ColorSensor JewelColor;
    public DigitalChannel DumperTouchSensorRight;
    public DigitalChannel DumperTouchSensorLeft;
    public DistanceSensor IntakeDistance;
    public DistanceSensor FlipperDistance2;
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
    double JewelServoDistancePos = .375;
    double JewelServoDownPos = .14; //.2 really
    double RegularTurnSpeed = .165;
    double IntakeSpeed = -.7;
    double CryptoboxServoInPos = 0;
    double CryptoboxServoOutPos = 1;
    double CryptoboxServoMidPos = .65;
    double programStartOrientation;
    double stayOnHeading = 84.17;

    int currentColumn = 0;
    boolean LeftColumnPlaced = false;
    boolean CenterColumnPlaced = false;
    boolean RightColumnPlaced = false;


    boolean knockedCryptoboxSideJewel = false;

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
        ConveyorLeft = hardwareMap.dcMotor.get("ConveyorLeft");
        ConveyorRight = hardwareMap.dcMotor.get("ConveyorRight");
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
        Blocker = hardwareMap.servo.get("Blocker");
        JewelArm = hardwareMap.servo.get("JewelServo");
        CryptoboxServo = hardwareMap.servo.get("CryptoboxServo");
        IntakeServo = hardwareMap.servo.get("IntakeServo");
        FlipperServo = hardwareMap.servo.get("FlipperServo");

        // Initialize and hardware map Sensors
        DumperTouchSensorRight = hardwareMap.get(DigitalChannel.class, "DumperTouchSensorRight");
        DumperTouchSensorLeft = hardwareMap.get(DigitalChannel.class, "DumperTouchSensorLeft");
        DumperTouchSensorRight.setMode(DigitalChannel.Mode.INPUT);
        DumperTouchSensorLeft.setMode(DigitalChannel.Mode.INPUT);
        CryptoboxDistance = hardwareMap.get(DistanceSensor.class, "CryptoboxSensor");
        BackDistance = hardwareMap.get(I2CXLv2.class, "BackDistance");
        JewelColor = hardwareMap.get(ColorSensor.class, "JewelSensor");

        //IntakeDistance = hardwareMap.get(DistanceSensor.class, "IntakeSensor");
        IntakeDistance = hardwareMap.get(DistanceSensor.class, "FlipperSensor1");
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
        telemetry.addData("Vumark FOUND!!!!", CryptoKey);
        telemetry.addData("Good To Run", 1);
        telemetry.update();
        LinearSlideMotor.setPower(0);
        vuforiaHardware.Stop();
    }

    // Start Movement methods
    public void drive(double speed, int direction, double time){
        double startingHeading = getHeading();
        double timeStarted = runtime.time();
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(opModeIsActive() && runtime.time() - timeStarted < time) {
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
            while(opModeIsActive() && notAtTarget) {

                if (Math.abs(target) - Math.abs(FrontLeft.getCurrentPosition()) > 25 && runtime.seconds() < 28.5 && (startTime + timeout > runtime.seconds())) {
                    gyroDrive(Heading, speed, direction);
                    //smartIntake(); Conveyor/intake
                }else{
                    notAtTarget = false;
                }
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

            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            double power = 0;
            double error;


            target = FrontLeft.getCurrentPosition() + (int) (inches * CountsPerInch * direction);
            double decelTicks = (int) (decelInches * CountsPerInch);

            while(Math.abs(target) - Math.abs(FrontLeft.getCurrentPosition()) > 50 && runtime.seconds() < 28.5 && (startTime + timeout > runtime.seconds())) {
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
                    DistanceToTravel = 4;
                    //4?
                } else if (PylonsToFind == 1) {
                    DistanceToTravel = 12;

                } else if (PylonsToFind == 2) {
                    //18?
                    DistanceToTravel = 20;
                }
            }
        }else{
            if(Direction == Forward){
                //red
                if (PylonsToFind == 0) {
                    DistanceToTravel = 2;
                } else if (PylonsToFind == 1) {
                    DistanceToTravel = 10.5;
                } else if (PylonsToFind == 2) {
                    DistanceToTravel = 18;
                }
            }else{
                if (PylonsToFind == 0) {
                    DistanceToTravel = 0;
                    //4?
                } else if (PylonsToFind == 1) {
                    DistanceToTravel = 8;

                } else if (PylonsToFind == 2) {
                    //18?
                    DistanceToTravel = 18;
                }
            }
        }

        if(startingPosition == 1 || startingPosition == 4) {
            if (Direction == Reverse) {
                if (knockedCryptoboxSideJewel) {
                    EncoderDriveAccelDecel(.5, 10 + DistanceToTravel, 12, Direction, stayOnHeading, 3 );
                } else {
                    EncoderDriveAccelDecel(.5, 13 + DistanceToTravel, 12, Direction,stayOnHeading, 3);
                }
            } else {
                if (knockedCryptoboxSideJewel) {
                    EncoderDriveAccelDecel(.5, 10 + DistanceToTravel, 12,Direction, stayOnHeading, 3);
                } else {
                    EncoderDriveAccelDecel(.5, 16 + DistanceToTravel, 12,Direction, stayOnHeading, 3);
                }
            }
        }else{
            EncoderDriveAccelDecel(.5,  DistanceToTravel, 10, Forward, stayOnHeading, 5);
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
                goToDistance(.3, 59, BackDistance, 2, 1);
            }else{
                //blue side, far stone
                goToDistance(.3, 57, BackDistance, 2, 1);
            }
        }
        driveToCrypotboxEncoders(Direction, startingPosition);

        stopDriveMotors();
        turnToCryptobox(startingPosition);
        if(Direction == Reverse) {
            if(startingPosition == 1) {
                drive(.25, Reverse, 1);
            }else{
                drive(.2, Reverse, .75);
            }
        }else{
            if(startingPosition == 4) {
                drive(.325, Reverse, 1.25);
            }else{
                drive(.2, Reverse, .75);
            }

        }
        extendCryptoboxArmForFirstGlyph();
        findColumn(1.5);
        stopDriveMotors();
        placeByFlippingFirstGlyph(3);


    }
    public void endAuto(){
        JewelArm.setPosition(JewelServoDistancePos);
        drive(1, Forward, .125);
        CryptoboxServo.setPosition(CryptoboxServoInPos);
        sleep(200);
        JewelArm.setPosition(JewelServoUpPos);
        telemetry.addData("Vumark", CryptoKey);
        telemetry.addData("Color", color);
        telemetry.addData("No Glyphs", glyphs);
        telemetry.update();
        sleep(3000);

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
                moveBy(.015, .3, 0); //moveBy is a function that handles robot movement
            }else if(CryptoboxDistance.getDistance(DistanceUnit.CM) < 7.15){
                FoundPylon = true;
            }else {
                moveBy(.015, -.3, 0); //moveBy is a function that handles robot movement
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
        double startHeading = getHeading();
        boolean foundWall = false;
        double timeout = runtime.seconds() + Timeout;
        while (opModeIsActive() && !foundWall && (runtime.seconds() < timeout)) {
            int ThisLoopDistance = BackDistance.getDistance();
            if(ThisLoopDistance > 200 || ThisLoopDistance < 21){
                //sensor val is bad, skip this loop
                moveBy(speed, 0, 0);
            }else if(BackDistance.getDistance() > distance){
                moveBy(speed, 0, 0);
            }else{
                stopDriveMotors();
                foundWall = true;
            }
        }
    }
    public void goToDistance(double targetSpeed, double distance,  I2CXLv2 distanceSensor, double timeout, int tolerance){
        double startTime = runtime.seconds();
        double maxTime = startTime + timeout;
        double startHeading = getHeading();
        boolean foundTarget = false;
        while (opModeIsActive() && !foundTarget && maxTime - runtime.seconds() > .1) {
            int ThisLoopDistance = distanceSensor.getDistance();
            double error = distance - ThisLoopDistance;
            if(ThisLoopDistance > 500 || ThisLoopDistance < 21){
                stopDriveMotors();
                //sensor val is bad, stop bot so it doesn't go too far
            }else if(ThisLoopDistance > distance + tolerance || ThisLoopDistance < distance - tolerance){
                int Direction = (int) Range.clip(error, -1, 1);
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
        if(!columnsPlaced[0]){
            columnToGoTo = 0;
        }else if(!columnsPlaced[1]){
            columnToGoTo = 1;
        }else if(!columnsPlaced[2]){
            columnToGoTo = 2;
        }else{
            //nowhere left to place
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
            if(direction == Reverse){
                if(columnsToGo > 0){
                    strafingLeft = true;
                }else{
                    strafingLeft = false;
                }
                //blue side
                if(strafingLeft){
                    strafeTime = Math.abs(columnsToGo/2) - .2;
                }else{
                    strafeTime = Math.abs(columnsToGo/2) + .2;
                }
            }else {
                if (columnsToGo > 0) {
                    strafingLeft = false;
                } else {
                    strafingLeft = true;
                }
                //red side
                if (strafingLeft) {
                    strafeTime = Math.abs(columnsToGo/2) + .2;
                    //make sure this columnsToGo isn't returning 0
                } else {
                    strafeTime = Math.abs(columnsToGo/2) - .2;
                }
            }
        }
            //}
            //Put in other function, just to see if it's the movement that's handled by this function that's out of whack
            //Y speed, x speed, rotation speed, time
        driveWStrafe(-.15, .4*strafingDirectionMultiplier, 0, strafeTime);

        telemetry.addData("Strafe Time", strafeTime);
        telemetry.addData("Strafing Left?", strafingLeft);
        telemetry.addData("Strafing Multiplier", strafingDirectionMultiplier);
        telemetry.addData("Columns To Go", columnsToGo);
        telemetry.addData("Columns placed", columnsPlaced);
        telemetry.update();
    }

    public void ramThePitRelicSide(int startingPosition, int direction){
        //Drive forward to pit
        // get glyphs
        // drive back to close to cryptobox
        // go to correct column
        // ram wall
        // find column
        // place glyph

        //Drive forward to pit - Should be complete
        FlipperServo.setPosition(FlipperServoDownPos);
        CryptoboxServo.setPosition(CryptoboxServoMidPos);
        ConveyorLeft.setPower(1);
        ConveyorRight.setPower(1);
        EncoderDrive(.95, 28, Forward, stayOnHeading, .75);

        //Grab glyphs, this part needs work (also hardware side tho)
        driveToGlyphs(startingPosition, 16);
        FlipperServo.setPosition(FlipperServoUpPos);

        //Drive back to cryptobox, this part needs a little work, maybe use findwall instead of goToDistance
        EncoderDrive(.75, 20, Reverse, stayOnHeading, 2);
        goToDistance(.75, 45, BackDistance, 1, 5);
        ConveyorLeft.setPower(-1);
        ConveyorRight.setPower(-1);
        //go to correct column - need some work, just strafeToColumn reliability/distances/power values
        strafeToColumnMG(direction);
        drive(.2, Reverse, .5);
        extendCryptoboxArmForFirstGlyph();
        EncoderDrive(.25, 3, Reverse, stayOnHeading, 1);

        //Find column and place glyphs - complete
        findColumn(1);
        stopDriveMotors();
        placeByFlippingSecondGlyph(3);
        //no more glyphs, end auton
        endAuto();

    }
    /*public void ramThePitRelicSide(int startingPosition, int direction){
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FlipperServo.setPosition(FlipperServoDownPos);
        IntakeServo.setPosition(IntakeServoDown);
        EncoderDrive(.95, 18,  Forward, stayOnHeading, 5);
        ConveyorLeft.setPower(1);
        ConveyorRight.setPower(1);
        CryptoboxServo.setPosition(CryptoboxServoMidPos);
        driveToGlyphs(startingPosition, 6);
        turnToCryptobox(startingPosition);
        CryptoboxServo.setPosition(CryptoboxServoMidPos);
        double time =.5;
        findWall(-1, 55, 3);
        //Add for which columns it goes which next column
        int PylonsToFind = cryptoboxPylonsToGo(direction);

        *//*if(PylonsToFind == 1) {
            if(direction == Forward){
                gyroTurn(turningSpeed, -140);
            }else{
                gyroTurn(turningSpeed, -150);
            }
            EncoderDrive(.35, 10, Reverse, stayOnHeading, 1);
        }else if (PylonsToFind == 0){
            if(direction == Forward){
                gyroTurn(turningSpeed, -110);
            }else {
                //blue
                gyroTurn(turningSpeed, -100);
            }
            EncoderDrive(.35, 15, Reverse, stayOnHeading, 1);
        }else if (PylonsToFind == 2){
            if(direction == Forward){
                gyroTurn(turningSpeed, -105);
            }else {
                //blue
                gyroTurn(turningSpeed, -100);
            }
            EncoderDrive(.75, 15, Reverse, stayOnHeading, 1);
        }*//*
        turnToCryptobox(startingPosition);
        findWall(-.35, 35, 3);
        driveWStrafe(-.2, 0, 0, .5);

        turnToCryptobox(startingPosition);
        extendCryptoboxArmForFirstGlyph();
        EncoderDrive(.15, 3.15, Reverse, stayOnHeading, 1);
        findColumn(1.25);
        stopDriveMotors();
        placeByFlipping(2);
        // add if time < needed time go back
        // else pick up another?
    }*/

    public void ramThePitTeamSide(int startingPosition, int direction){
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double startingRotation = getHeading();
        double angleMultiplier = cryptoboxPylonsToGo(-direction);
        EncoderDrive(.175, 6, Forward, stayOnHeading, 2);
        double turningAngle = 0;
        if( startingPosition == 2){
            turningAngle = -25 - 10*angleMultiplier;
        }else{
            turningAngle =  -155 + 10*angleMultiplier;
        }
        gyroTurn(turningSpeed, turningAngle);
        EncoderDrive(.95, 30, Forward, stayOnHeading, 2.5);
        Blocker.setPosition(BlockerServoUp);
        CryptoboxServo.setPosition(CryptoboxServoOutPos);
        //driveToGlyphs(startingPosition, 6);
        CryptoboxServo.setPosition(CryptoboxServoMidPos);
        gyroTurn(turningSpeed, turningAngle);
        int PylonsToFind = cryptoboxPylonsToGo(direction);

        if(PylonsToFind == 2){
            EncoderDrive(.85, 28, Reverse, stayOnHeading, 4);
        }else {
            EncoderDrive(.85, 24, 6, stayOnHeading, 4);
        }
        turnToCryptobox(startingPosition);
        driveWStrafe(-.15, 0, 0, .75);

        turnToCryptobox(startingPosition);
        extendCryptoboxArmForFirstGlyph();
        if(PylonsToFind == 0){
            placeByFlippingSecondGlyph(3);
        }else {
            EncoderDrive(.15, 3.15, Reverse, stayOnHeading, 1);
            findColumn(1.25);
            stopDriveMotors();
            placeByFlippingSecondGlyph(3);
        }
        // add if time < needed time go back
        // else pick up another?
    }

    public void driveToGlyphs(int startingPosition, int inchesToGo){
        double timer;
        boolean wentLeft = false;
        boolean wentRight = false;
        boolean gotGlyph = false;
        while(runtime.seconds() < 25 && !gotGlyph && opModeIsActive()){
            boolean linedUp = false;
            FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            double startingEncoderCount = FrontLeft.getCurrentPosition();
            double limitEncoderCount = startingEncoderCount + inchesToGo*CountsPerInch;

            while(!haveGlyph() && opModeIsActive() && runtime.seconds() < 22 && (Math.abs(FrontLeft.getCurrentPosition()) < Math.abs(limitEncoderCount)) )  {
                moveBy(.15, 0,0);
                smartIntake();
            }
            //intakeGlyph();
            glyphs += 1;
            //EncoderDrive(.75, 10, Reverse, stayOnHeading, 1.5);
            gotGlyph = true;
        }
    }
    public boolean haveGlyph(){
        boolean haveGlyph = false;
        double SensorVal = FlipperDistance2.getDistance(DistanceUnit.CM);
        if (SensorVal <= 50) {
            haveGlyph = true;
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
    /* public void intakeGlyphs(int inches){
         boolean haveGlyph = false;
         int color = 0;
         double startingHeading = getHeading();
         FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         double startingEncoderCount = FrontLeft.getCurrentPosition();
         double limitEncoderCount = startingEncoderCount + inches*CountsPerInch;
         DumpConveyor.setPower(1);
         ConveyorLeft.setPower(1);
         ConveyorRight.setPower(1);
         TopIntakeServoLeft.setPower(1);
         TopIntakeServoRight.setPower(1);
         while(!haveGlyph && (Math.abs(FrontLeft.getCurrentPosition()) < Math.abs(limitEncoderCount)) && runtime.seconds() < 24){

             gyroDrive(startingHeading, .2, Forward);
             double SensorVal = IntakeDistance.getDistance(DistanceUnit.CM);
             if (SensorVal <= 14) {
                 IntakeServoLeft.setPower(IntakeSpeed);
                 IntakeServoRight.setPower(-IntakeSpeed);
                 haveGlyph = true;
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
             telemetry.addData("CurrentPos", FrontLeft.getCurrentPosition());
             telemetry.addData("Limit Pos", limitEncoderCount);
             telemetry.addData("Difference", Math.abs(limitEncoderCount) - Math.abs(FrontLeft.getCurrentPosition()));
             telemetry.update();
             //if color is > whatever, it's brown. Otherwise it's grey
         }
         stopDriveMotors();
         double timeLimit = runtime.time(TimeUnit.SECONDS);
         while(runtime.seconds() - timeLimit < .5){
             double SensorVal = IntakeDistance.getDistance(DistanceUnit.CM);
             if (SensorVal <= 11) {
                 IntakeServoLeft.setPower(IntakeSpeed);
                 IntakeServoRight.setPower(-IntakeSpeed);
                 haveGlyph = true;
             }else if(SensorVal > 11 && SensorVal < 20){
                 IntakeServoLeft.setPower(IntakeSpeed);
                 IntakeServoRight.setPower(IntakeSpeed);
             }else if (SensorVal >= 20){
                 IntakeServoLeft.setPower(-IntakeSpeed);
                 IntakeServoRight.setPower(-IntakeSpeed);
             }else{
                 IntakeServoLeft.setPower(IntakeSpeed);
                 IntakeServoRight.setPower(-IntakeSpeed);
             }
             //if color is > whatever, it's brown. Otherwise it's grey
         }
         if(IntakeColor.alpha() > 130){
             telemetry.addData("Grey", IntakeColor.alpha());
             color = 1;
         }else{
             telemetry.addData("Brown", IntakeColor.alpha());
             color = 2;
         }
         telemetry.update();
         double time = runtime.time(TimeUnit.SECONDS);
         while(runtime.seconds() - time < 1){
             double SensorVal = IntakeDistance.getDistance(DistanceUnit.CM);
             if (SensorVal <= 11) {
                 IntakeServoLeft.setPower(IntakeSpeed);
                 IntakeServoRight.setPower(-IntakeSpeed);
                 haveGlyph = true;
             }else if(SensorVal > 11 && SensorVal < 20){
                 IntakeServoLeft.setPower(IntakeSpeed);
                 IntakeServoRight.setPower(IntakeSpeed);
             }else if (SensorVal >= 20){
                 IntakeServoLeft.setPower(-IntakeSpeed);
                 IntakeServoRight.setPower(-IntakeSpeed);
             }else{
                 IntakeServoLeft.setPower(IntakeSpeed);
                 IntakeServoRight.setPower(-IntakeSpeed);
             }
             //if color is > whatever, it's brown. Otherwise it's grey
         }
         IntakeServoLeft.setPower(IntakeSpeed);
         IntakeServoRight.setPower(-IntakeSpeed);
         double inchesToDrive = FrontLeft.getCurrentPosition()/CountsPerInch;
         EncoderDrive(1, Math.abs(inchesToDrive), Reverse, stayOnHeading, 5);
     }
    */ // End movement methods
    // Motor and servo methods
    public void knockOffJewel(String AllianceColor, int startingPosition){
        JewelArm.setPosition(JewelServoDownPos);
        CryptoboxServo.setPosition(CryptoboxServoOutPos);
        telemetry.addData("KnockingJewel", 10);
        telemetry.update();
        // sleep to allow the jewel arm to go down
        sleep(750);
        //Knock off the jewel.  If the jewel is the way we go to get the cryptobox, we drive forward
        // To knock off, otherwise we turn.  This is to
        int Direction = jewelDirection(AllianceColor);
        if(startingPosition == 1){
            if(Direction == Forward) {
                knockedCryptoboxSideJewel = true;
                EncoderDrive(.35, 5, Reverse, stayOnHeading, 5);
            }else{
                double TurningAngle = 4 * Direction;
                gyroTurn(turningSpeed, TurningAngle);
            }
        }else if (startingPosition == 4){
            if(Direction == Reverse) {
                knockedCryptoboxSideJewel = true;
                EncoderDrive(.35, 5, Forward, stayOnHeading, 5);
            }else{
                double TurningAngle = 4 * Direction;
                gyroTurn(turningSpeed, TurningAngle);
            }
        }else{
            double TurningAngle = 4 * Direction;
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
    /* public void placeGlyph(RelicRecoveryVuMark Column){
         EncoderDrive(.15, .5, Forward, stayOnHeading, 2);
         Blocker.setPosition(BlockerServoDown);
         //findColumn();
         sleep(1000);
         EncoderDrive(.15,  5, Forward, stayOnHeading, 2);
         drive(-.15, Reverse, .5);
         CryptoboxServo.setPosition(CryptoboxServoMidPos);
         drive(.2, Reverse, 1);
     }
     public void placeGlyphTeamside(){
         Blocker.setPosition(BlockerServoDown);
         //findColumn();
         sleep(500);
         EncoderDrive(.25,  6, Forward, stayOnHeading, 2);
         CryptoboxServo.setPosition(CryptoboxServoMidPos);
         drive(.2, Reverse, 1);
     }
     public void placeSecondGlyphTeamside(){
         Blocker.setPosition(BlockerServoDown);
         //findColumn();
         sleep(500);
         drive(.2, Reverse, .5);
         EncoderDrive(.25,  8, Forward, stayOnHeading, 2);
         CryptoboxServo.setPosition(CryptoboxServoMidPos);
         while(runtime.seconds() < 28.75) {
             moveBy(-.35, 0, 0 );
         }
     }
     public void placeSecondGlyph() {
         EncoderDrive(.15, 1.5, Forward, stayOnHeading, 1);
         Blocker.setPosition(BlockerServoDown);
         //findColumn();
         sleep(500);
         drive(.2, Reverse, .5);
         sleep(500);
         EncoderDrive(.15, 6, Forward, stayOnHeading, 2);
         CryptoboxServo.setPosition(CryptoboxServoMidPos);
         drive(.2, Reverse, .5);
         drive(.75, Forward, .2);
         drive(.2, Reverse, .5);
     }*/
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
        EncoderDrive(.35, 1, Forward, stayOnHeading, 2);
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
                moveBy(.05, 0, 0);
            }
            placed = true;
            stopDriveMotors();
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
    public void pushInFirstGlyph (){
        drive(.4, Reverse, .5);
    }
    public void smartIntake(){
        double intakeValLeft = 20;
        double speed = 1;
        double SensorVal = IntakeDistance.getDistance(DistanceUnit.CM);
        if (SensorVal <= intakeValLeft && SensorVal > 6) {
            ConveyorRight.setPower(speed);
            ConveyorLeft.setPower(speed);
        }else if(SensorVal > intakeValLeft){
            ConveyorRight.setPower(speed);
            ConveyorLeft.setPower(0);
        }else if (SensorVal <= 5.5) {
            ConveyorRight.setPower(0);
            ConveyorLeft.setPower(speed);
        }else {
            ConveyorRight.setPower(speed);
            ConveyorLeft.setPower(speed);
        }
    }

}