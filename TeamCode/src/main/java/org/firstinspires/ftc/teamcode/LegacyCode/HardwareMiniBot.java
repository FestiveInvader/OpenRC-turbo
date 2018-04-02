package org.firstinspires.ftc.teamcode.LegacyCode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareMiniBot
{
    /* Declare OpMode members. */
    // This section declares hardware for the program, such as DC Motors, servos and sensors
    public DcMotorSimple DriveLeft = null; // Andymark NeveRest 3.7
    public DcMotorSimple DriveRight = null; // Andymark NeveRest 3.7

    public Servo PhoneServo = null;


    public ColorSensor ColorSensorLineLeft;
    public ColorSensor ColorSensorLineRight;
    public com.qualcomm.robotcore.hardware.GyroSensor GyroSensor;

    public ElapsedTime runtime = new ElapsedTime();

    // Final vaiables used mostly in methods
    public static final double CountsPerRev = 1120;    // Andymark NeveRest 40
    public static final double WheelDiameterInches = 4.0;     // For figuring circumference
    public static final double CountsPerInch = (CountsPerRev / (WheelDiameterInches * 3.1415));

    public static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    public static final double P_TURN_COEFF = .2;     // Larger is more responsive, but also less stable
    public static final double P_DRIVE_COEFF = .15;     // Larger is more responsive, but also less stable
    public static final double DistanceWanted = 24;

    public static final double SideBeaconInPos = .235;
    public static final double SideBeaconOutPos = .525;
    public static final double LeftBeaconInPos = .31;
    public static final double LeftBeaconOutPos = .05;
    public static final double RightBeaconInPos = .3;
    public static final double RightBeaconOutPos = .65;

    public static final double GyroTurnSpeed = .115;//.1/.125?
    public static final double SlowDriveSpeed = 0.4;
    public static final double FastDriveSpeed = 0.8;
    public static final double TurnSpeed = 0.5;
    public static final int Forward = 1;
    public static final int Reverse = -1;

    public static double LeftPower = 0;
    public static double RightPower = 0;

    HardwareMap MiniBotMap           =  null;
    private ElapsedTime period  = new ElapsedTime();


    /* Constructor */
    public HardwareMiniBot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap HardwareMiniBot) {

        // Save reference to Hardware map
        MiniBotMap = HardwareMiniBot;


        //This section declares the different motors on the robot, as well as their default
        // These are the motor hardware map maps
        DriveLeft   = HardwareMiniBot.dcMotor.get("DriveLeft");
        DriveRight = HardwareMiniBot.dcMotor.get("DriveRight");
        DriveLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        PhoneServo = HardwareMiniBot.servo.get("PhoneServo");


        // This section is declaring the servos used on the robot.


        // This section is declaring the sensors used on the robot.
        /*ColorSensorLineRight = HardwareMiniBot.colorSensor.get("ColorSensorLineFront");
        ColorSensorLineRight.setI2cAddress(I2cAddr.create8bit(0x3c));
        ColorSensorLineRight.enableLed(true);

        ColorSensorLineLeft = HardwareMiniBot.colorSensor.get("ColorSensorLineBack");
        ColorSensorLineLeft.setI2cAddress(I2cAddr.create8bit(0x4c));
        ColorSensorLineLeft.enableLed(true);

        ColorSensorLineRight.enableLed(false);
        ColorSensorLineLeft.enableLed(false);



        ColorSensorLineRight.enableLed(false);
        ColorSensorLineLeft.enableLed(false);
        ColorSensorLineRight.enableLed(true);
        ColorSensorLineLeft.enableLed(true);*/
    }


    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }

}


