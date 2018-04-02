package org.firstinspires.ftc.teamcode.LegacyCode;

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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Autonomous.I2CXLv2;

//@Author Eric Adams, Team 8417 'Lectric Legends

public class LegacyFunctions extends LinearOpMode {
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

    public DistanceSensor IntakeDistance;
    public DistanceSensor ConveyorDistance;
    public DistanceSensor CryptoboxDistance;
    public ColorSensor ConveyorColor;
    public ColorSensor JewelColor;
    public DigitalChannel DumperTouchSensorRight;
    public BNO055IMU IMU;
    public I2CXLv2 BackDistance;
    public I2CXLv2 RightDistance;

    // Variables used  in functions
    double CountsPerRev = 537.6;    // Andymark NeveRest 20 encoder counts per revolution
    double GearRatio = .889;
    double WheelDiameterInches = 4.0;     // For figuring circumference
    double CountsPerInch = ((CountsPerRev / (WheelDiameterInches * 3.1415))*GearRatio);
    double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    double P_TURN_COEFF = .2;     // Larger is more responsive, but also less stable
    double P_DRIVE_COEFF = .15;     // Larger is more responsive, but also less stable
    double turningSpeed = .225;
    //empty is 0, grey is 1, brown is 2,
    int[] CurrentCryptobox = new int[]{0,0,0,0,0,0,0,0,0,0,0,0};
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


    double BlockerServoUp = .35;
    double BlockerServoDown = .56;
    double JewelServoUpPos = .61;
    double JewelServoDistancePos = .34;
    double JewelServoDownPos = .14; //.2 really
    double RegularTurnSpeed = .165;
    double IntakeSpeed = 1;
    double CryptoboxServoInPos = .1;
    double CryptoboxServoOutPos = .985;
    double CryptoboxServoMidPos = .65;

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
        RightDistance = hardwareMap.get(I2CXLv2.class, "RightDistance");
        BackDistance = hardwareMap.get(I2CXLv2.class, "BackDistance");
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
        while(CryptoKey == RelicRecoveryVuMark.UNKNOWN && runtime.seconds() < 1){
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

   /* public void EncoderDriveWAccelDecel(double speed, double Inches, double accelerationInches, double decelerationInches, int Direction) {
        // Declares variables that are used for this method
        int NewLeftTarget;
        int NewRightTarget;
        int FrontLeftPosition;
        int FrontRightPosition;
        int BackLeftPosition;
        int BackRightPosition;
        int direction = -Direction;

        double DecelTicks;
        double Speed = speed;
        double LeftSpeed;
        double RightSpeed;
        double DistanceBeforeDeceleration;
        double DecelTickMultiplier;
        double DecelSpeedVar;
        double MinSpeed = .05;
        double SpeedToDecelerate;

        boolean Running = true;
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
            NewLeftTarget = FrontLeft.getCurrentPosition() + (int) (Inches * CountsPerInch);
            NewRightTarget = FrontRight.getCurrentPosition() + (int) (Inches * CountsPerInch);
            // Gets the current position of the encoders at the beginning of the EncoderDrive method
            FrontLeftPosition = FrontLeft.getCurrentPosition();
            FrontRightPosition = FrontRight.getCurrentPosition();
            BackLeftPosition = BackLeft.getCurrentPosition();
            BackRightPosition = BackRight.getCurrentPosition();


            // Setup for deceleration
            DecelTicks = ((decelerationInches * CountsPerInch));
            SpeedToDecelerate = speed - MinSpeed;
            DistanceBeforeDeceleration = Math.abs(NewLeftTarget) - Math.abs(DecelTicks);
            DecelTickMultiplier = (SpeedToDecelerate/DecelTicks);

            // Gives the encoders the target.
            FrontLeft.setTargetPosition(NewLeftTarget);
            FrontRight.setTargetPosition(NewRightTarget);
            BackLeft.setTargetPosition(NewLeftTarget);
            BackRight.setTargetPosition(NewRightTarget);
            // This gets where the motor encoders will be at full position when it will be at full speed.
            double LeftEncoderPositionAtFullSpeed = ((accelerationInches*(CountsPerInch)) + FrontLeftPosition);
            double RightEncoderPositionAtFullSpeed = ((accelerationInches*(CountsPerInch)) + FrontRightPosition);

            // This gets the absolute value of the encoder positions at full speed - the current speed, and while it's greater than 0, it will continues increasing the speed.
            // This allows the robot to accelerate over a set number of inches, which reduces wheel slippage and increases overall reliability
            while (opModeIsActive() && Running) {// && opModeIsActive
                // While encoders are not at position
                if ((Math.abs(speed) - (Math.abs(FrontLeft.getPower())) > .05)){

                    LeftSpeed = (Range.clip((Math.abs(FrontLeft.getCurrentPosition())/ Math.abs(LeftEncoderPositionAtFullSpeed)), MinSpeed, speed));
                    telemetry.addData("Accelerating Encoders", 1);
                    telemetry.update();
                    // This allows the robot to accelerate over a set distance, rather than going full speed.  This reduces wheel slippage and increases reliability.
                }else if(Math.abs(FrontLeft.getCurrentPosition()) > Math.abs(DistanceBeforeDeceleration)){

                    // Ramp down the power
                    DecelSpeedVar = ((Math.abs(FrontLeft.getCurrentPosition())-Math.abs(DistanceBeforeDeceleration)));
                    double DecelClipVar = DecelSpeedVar* Math.abs(DecelTickMultiplier);
                    LeftSpeed = Range.clip(((Math.abs(speed) - Math.abs(DecelClipVar))), MinSpeed, speed);
                    telemetry.addData("Decel Encoders", 1);
                    telemetry.update();
                }else{
                    telemetry.addData("Normal Encoders", 1);
                    telemetry.update();
                    RightSpeed = speed;
                    LeftSpeed = speed;
                }
                if(Math.abs(NewLeftTarget) - Math.abs(FrontLeft.getCurrentPosition()) < -1) {
                    //If absolute value of wanted encoder count at finish -
                    // the absolute value of current motor is < -1, then stop running.
                    Running = false;
                }
                FrontLeft.setPower(Range.clip(Math.abs(LeftSpeed), MinSpeed*direction,direction*speed));
                FrontRight.setPower(Range.clip(Math.abs(LeftSpeed), MinSpeed*direction,direction*speed));
                BackLeft.setPower(Range.clip(Math.abs(LeftSpeed), MinSpeed*direction,direction*speed));
                BackRight.setPower(Range.clip(Math.abs(LeftSpeed), MinSpeed*direction,direction*speed));
                telemetry.addData("Setting power", LeftSpeed);
                telemetry.update();
            }

            // Stops all motion
            // Set to run without encoder, so it's not necessary to declare this every time after the method is used
            FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // Set power to 0
            stopDriveMotors();
            telemetry.addData("Over", 1);
            telemetry.update();
        }
    }
    public void EncoderTurn(double speed, double leftInches, double rightInches, int Direction) {
        // Declares variables that are used for this method
        int NewLeftTarget;
        int NewRightTarget;
        double MinSpeed = .1;
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
    public void fieldOriented(double y, double x, double c, double gyroheading) {
        // X and Y are left joy, C is rotation of right joy
        double cosA = Math.cos(Math.toRadians(gyroheading));
        double sinA = Math.sin(Math.toRadians(gyroheading));
        double xOut = x * cosA - y * sinA;
        double yOut = x * sinA + y * cosA;
        moveBy(yOut, xOut, c);
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
    }*/

}