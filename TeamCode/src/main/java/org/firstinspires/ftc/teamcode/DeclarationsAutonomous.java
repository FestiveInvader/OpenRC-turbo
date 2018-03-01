package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
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
import java.util.concurrent.TimeUnit;

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
    public DcMotor LinearSlideMotor = null;           // NeveRest 20

    // Declare Servos
    public CRServo IntakeServoLeft = null;  // VEX 393
    public CRServo IntakeServoRight = null; // VEX 393
    public CRServo TopIntakeServoLeft = null;
    public CRServo TopIntakeServoRight = null;
    public CRServo DumpConveyor = null;     // Rev SRS
    public Servo Blocker = null;            // Rev SRS  Heh, block-er
    public Servo JewelArm = null;           // Rev SRS
    public Servo CryptoboxServo = null;

    public DistanceSensor IntakeDistance;
    public DistanceSensor ConveyorDistance;
    public DistanceSensor CryptoboxDistance;
    public DistanceSensor FrontLeftDistance;
    public DistanceSensor FrontRightDistance;
    public ColorSensor IntakeColor;
    public ColorSensor JewelColor;
    public DigitalChannel DumperTouchSensorRight;
    public BNO055IMU IMU;
    public I2CXLv2 BackDistance;
    public I2CXLv2 RightDistance;

    // Variables used  in functions
    double CountsPerRev = 537.6;    // Andymark NeveRest 20 encoder counts per revolution
    double GearRatio = .889;
    double WheelDiameterInches = 4.0;     // For figuring circumference
    double CountsPerInch = ((CountsPerRev / ((WheelDiameterInches * 3.1415))/GearRatio));
    double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    double P_TURN_COEFF = .2;     // Larger is more responsive, but also less stable
    double P_DRIVE_COEFF = .15;     // Larger is more responsive, but also less stable
    double turningSpeed = .215;
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
    int DumpingDegreesOfTravel = 85; // Wanted degrees of the dump to travel
    int FractionOfRevolutionToDump = 360/DumpingDegreesOfTravel;
    int DumpingMotorEncoderTicks = 1680; // NeveRest 60
    int DumpingGearRatio = DumpingGearDriving/DumpingGearDriven; // 2:1
    int DumpingEncoderTicksPerRevolution = DumpingMotorEncoderTicks*DumpingGearRatio;
    int glyphs;



    double BlockerServoUp = .35;
    double BlockerServoDown = .56;
    double JewelServoUpPos = .61;
    double JewelServoDistancePos = .34;
    double JewelServoDownPos = .14; //.2 really
    double RegularTurnSpeed = .165;
    double IntakeSpeed = -.7;
    double CryptoboxServoInPos = 0;
    double CryptoboxServoOutPos = 1;
    double CryptoboxServoMidPos = .65;
    double programStartOrientation;
    double stayOnHeading = 84.17;

    boolean knockedCryptoboxSideJewel = false;

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
    int color = 0;
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
        LinearSlideMotor = hardwareMap.dcMotor.get("LinearSlideMotor");

        DumpingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ConveyorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        ConveyorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LinearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        CryptoboxDistance = hardwareMap.get(DistanceSensor.class, "CryptoboxSensor");
        FrontLeftDistance = hardwareMap.get(DistanceSensor.class, "FrontLeftDistance");
        FrontRightDistance = hardwareMap.get(DistanceSensor.class, "FrontRightDistance");
        RightDistance = hardwareMap.get(I2CXLv2.class, "RightDistance");
        BackDistance = hardwareMap.get(I2CXLv2.class, "BackDistance");
        IntakeColor = hardwareMap.get(ColorSensor.class, "IntakeSensor");
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
        LinearSlideMotor.setPower(0);
        runtime.reset();
        while(CryptoKey == RelicRecoveryVuMark.UNKNOWN && runtime.seconds() < .15){
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
    public void driveWStrafe(double yspeed, double xspeed, double time){
        double startingHeading = getHeading();
        double timeStarted = runtime.time();
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(opModeIsActive() && runtime.time() - timeStarted < time) {
            moveBy(yspeed, xspeed, 0);
        }
        stopDriveMotors();
    }
    public void EncoderDrive(double speed, double Inches, int direction, double heading, double timeout) {
        double startTime = runtime.seconds();
        double Heading = 0;
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

            while (opModeIsActive() && Math.abs(target) - Math.abs(FrontLeft.getCurrentPosition()) > 25
                    && runtime.seconds() < 28.5 && (startTime + timeout > runtime.seconds())) {
                gyroDrive(Heading, speed, direction);
            }
            stopDriveMotors();
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
    public void gyroDrive(double targetAngle, double targetSpeed, int direction) {
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
    public void moveBy(double y, double x, double c) {
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
                    DistanceToTravel = 2;
                    //4?
                } else if (PylonsToFind == 1) {
                    DistanceToTravel = 10;

                } else if (PylonsToFind == 2) {
                    //18?
                    DistanceToTravel = 18;
                }
            }
        }

        if(startingPosition == 1 || startingPosition == 4) {
            if (Direction == Reverse) {
                if (knockedCryptoboxSideJewel) {
                    EncoderDrive(.2, 16 + DistanceToTravel, Direction, stayOnHeading, 5);
                } else {
                    EncoderDrive(.2, 21 + DistanceToTravel, Direction,stayOnHeading, 5);
                }
            } else {
                if (knockedCryptoboxSideJewel) {
                    EncoderDrive(.2, 16 + DistanceToTravel, Direction, stayOnHeading, 5);
                } else {
                    EncoderDrive(.2, 22 + DistanceToTravel, Direction, stayOnHeading, 5);
                }
            }
        }else{
            EncoderDrive(.2,  DistanceToTravel, Forward, stayOnHeading, 5);
        }
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

    public void driveToCryptoboxDistanceVariance(int Direction){
        int MinTol = 6;
        int MaxTol = 20;
        boolean foundPylon = false;
        double LastLoopDistance = RightDistance.getDistance();
        while (opModeIsActive() && !foundPylon) {
            int ThisLoopDistance = RightDistance.getDistance();
            if(ThisLoopDistance > 60 || ThisLoopDistance < 21 || LastLoopDistance > 60 || LastLoopDistance < 21){
                //sensor val is bad, skip this loop
                gyroDrive(0, .11, Direction);
            }else if(Math.abs(ThisLoopDistance - LastLoopDistance) >= MinTol &&
                    Math.abs(ThisLoopDistance - LastLoopDistance) <= MaxTol){
                foundPylon = true;
            }else{
                gyroDrive(0, .11, Direction);
            }
            telemetry.addData("This loop" , ThisLoopDistance);
            telemetry.addData("Last loop" , LastLoopDistance);
            telemetry.addData("Difference", Math.abs(ThisLoopDistance - LastLoopDistance));
            telemetry.addData("Vumark", CryptoKey);
            telemetry.update();
            LastLoopDistance = ThisLoopDistance;
        }
        stopDriveMotors();
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
        EncoderDrive(.2, 6.5, Forward, stayOnHeading, 5);
        sleep(300);
        EncoderDrive(.2, 3, Reverse, stayOnHeading, 2);
    }
    public void driveAndPlace(RelicRecoveryVuMark CryptoKey, int Direction, int Placement, double gyroOffset, int startingPosition){
        // Tolerance +- of the beginning distance, to account for small mistakes when setting robot up
        // and while knocking the jewel off
        if(startingPosition == 2 || startingPosition == 3){
            if(Direction == Forward){
                // Red side, far stone
                goToDistance(.3, 59, BackDistance, 2);
            }else{
                //blue side, far stone
                goToDistance(.3, 57, BackDistance, 2  );
            }
        }
        driveToCrypotboxEncoders(Direction, startingPosition);

        stopDriveMotors();
        turnToCryptobox(startingPosition);
        if(Direction == Reverse) {
            drive(.25, Reverse, 1);
        }else{
            drive(.325, Reverse, 1.15);

        }
        extendCryptoboxArmForFirstGlyph();
        findColumn(1.5);
        stopDriveMotors();
        placeGlyph(CryptoKey);
    }
    public void endAuto(){
        JewelArm.setPosition(JewelServoDistancePos);
        drive(.15, Forward, .75);
        CryptoboxServo.setPosition(CryptoboxServoInPos);
        sleep(200);
        JewelArm.setPosition(JewelServoUpPos);
        telemetry.addData("Vumark", CryptoKey);
        telemetry.addData("Color", color);
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
        // a pylon in that location, and we can assume our position from there
        boolean FoundPylon = false;
        while(opModeIsActive() && !FoundPylon && Timeout - runtime.seconds() > .1){
            if (CryptoboxDistance.getDistance(DistanceUnit.CM) < 8 ) {
                moveBy(.015, .425, 0); //moveBy is a function that handles robot movement
            }else if(CryptoboxDistance.getDistance(DistanceUnit.CM) < 10){
                FoundPylon = true;
            }else {
                moveBy(.015, -.425, 0); //moveBy is a function that handles robot movement
            }
        }
        stopDriveMotors();
    }
    public void findWall(double speed, double distance){
        double startHeading = getHeading();
        boolean foundWall = false;
        while (opModeIsActive() && !foundWall) {
            int ThisLoopDistance = BackDistance.getDistance();
            if(ThisLoopDistance > 200 || ThisLoopDistance < 21){
                //sensor val is bad, skip this loop
                gyroDrive(startHeading, .2, Reverse);
            }else if(BackDistance.getDistance() > distance){
                gyroDrive(startHeading, .2, Reverse);
            }else{
                stopDriveMotors();
                foundWall = true;
            }
        }
    }
    public void goToDistance(double targetSpeed, double distance,  I2CXLv2 distanceSensor, double timeout){
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
            }else if(ThisLoopDistance > distance + 1 || ThisLoopDistance < distance - 1){
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
    }
    public void ramThePitRelicSide(int startingPosition, int direction){
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        EncoderDrive(.95, 20,  Forward, stayOnHeading, 5);
        Blocker.setPosition(BlockerServoUp);
        DumpConveyor.setPower(1);
        CryptoboxServo.setPosition(CryptoboxServoMidPos);
        intakeGlyphs(24);
        EncoderDrive(.85, 18,  Reverse, stayOnHeading, 4);
        double time = .75;
        if(CryptoKey.equals(RelicRecoveryVuMark.CENTER)){
            time = 1.5;
        }

        driveWStrafe(0, .4, time);
        driveWStrafe(-.2, 0, .75);
        extendCryptoboxArmForFirstGlyph();
        EncoderDrive(.15, 2, Reverse, stayOnHeading, 1);
        findColumn(1);
        stopDriveMotors();

        placeSecondGlyph();
        // add if time < needed time go back
        // else pick up another?
    }
    public void ramThePitTeamSide(int startingPosition, int direction){
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double startingRotation = getHeading();
        double angleMultiplier = cryptoboxPylonsToGo(-direction);
        EncoderDrive(.2, 4, Forward, stayOnHeading, 2);
        double turningAngle = 0;
        if( startingPosition == 2){
            turningAngle = -25 - 8*angleMultiplier;
        }else{
            turningAngle =  -150 + 5*angleMultiplier;
        }
        gyroTurn(turningSpeed, turningAngle);
        EncoderDrive(.95, 36,  Forward, stayOnHeading, 2.5);
        Blocker.setPosition(BlockerServoUp);
        DumpConveyor.setPower(1);
        CryptoboxServo.setPosition(CryptoboxServoOutPos);
        intakeGlyphs(24);
        CryptoboxServo.setPosition(CryptoboxServoMidPos);
        gyroTurn(turningSpeed, turningAngle);
        EncoderDrive(.85, 34,  Reverse, stayOnHeading, 4);
        gyroTurn(turningSpeed, startingRotation);
        double time = .65;
        if(CryptoKey.equals(RelicRecoveryVuMark.CENTER)){
            time = 1;
        }
        driveWStrafe(-.2, -.4*direction, time);
        gyroTurn(turningSpeed, startingRotation);
        driveWStrafe(-.2, 0, .35);
        extendCryptoboxArmForFirstGlyph();
        findColumn(1);
        stopDriveMotors();
        placeSecondGlyph();
        // add if time < needed time go back
        // else pick up another?
    }
    public void intakeGlyphs(int inches){
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
    // End movement methods
    // Motor and servo methods
    public void knockOffJewel(String AllianceColor, int startingPosition){
        JewelArm.setPosition(JewelServoDownPos);
        CryptoboxServo.setPosition(CryptoboxServoOutPos);
        telemetry.addData("KnockingJewel", 10);
        telemetry.update();
        // sleep to allow the jewel arm to go down
        sleep(1000);
        //Knock off the jewel.  If the jewel is the way we go to get the cryptobox, we drive forward
        // To knock off, otherwise we turn.  This is to
        int Direction = jewelDirection(AllianceColor);
        if(startingPosition == 1){
            if(Direction == Forward) {
                knockedCryptoboxSideJewel = true;
                EncoderDrive(.5, 5, Reverse, stayOnHeading, 5);
            }else{
                double TurningAngle = 3 * Direction;
                gyroTurn(.25, TurningAngle);
            }
        }else if (startingPosition == 4){
            if(Direction == Reverse) {
                knockedCryptoboxSideJewel = true;
                EncoderDrive(.5, 5, Forward, stayOnHeading, 5);
            }else{
                double TurningAngle = 3 * Direction;
                gyroTurn(.25, TurningAngle);
            }
        }else{
            double TurningAngle = 3 * Direction;
            gyroTurn(.25, TurningAngle);
        }
        JewelArm.setPosition(JewelServoUpPos);
        // Turn back to the original robot orientation
        gyroTurn(RegularTurnSpeed, 0);
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
    public void placeGlyph(RelicRecoveryVuMark Column){
        EncoderDrive(.15, .5, Forward, stayOnHeading, 2);
        DumpConveyor.setPower(1);
        Blocker.setPosition(BlockerServoDown);
        //findColumn();
        sleep(1000);
        EncoderDrive(.15,  5, Forward, stayOnHeading, 2);
        drive(-.15, Reverse, .5);
        CryptoboxServo.setPosition(CryptoboxServoMidPos);
        drive(.2, Reverse, 1);
    }
    public void placeSecondGlyph(){
        EncoderDrive(.15, 3, Forward, stayOnHeading, 2);
        DumpConveyor.setPower(1);
        Blocker.setPosition(BlockerServoDown);
        //findColumn();
        sleep(1000);
        EncoderDrive(.15,  2, Forward, stayOnHeading, 2);
        drive(.2, Reverse, .5);
        CryptoboxServo.setPosition(CryptoboxServoMidPos);
        drive(.2, Forward, .5);
        drive(.2, Reverse, .5);
    }
}