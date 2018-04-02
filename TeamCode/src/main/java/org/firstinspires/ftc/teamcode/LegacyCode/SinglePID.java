package org.firstinspires.ftc.teamcode.LegacyCode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Michael on 7/29/2017.
 */

public class SinglePID extends LinearOpMode {

    //DcMotor leftSide;
    DcMotor rightSide;

    public static final double Kp = .0002;

    public static final double Ki = 0;

    public static final double Kd = 0;

    public int integral;

    public int dt;

    public double u;

    public int error;

    public int previousError;

    public int setPoint;

    public double rightPower;

    public int targetValue = 5000;

    @Override
    public void runOpMode() {
        //leftSide = hardwareMap.dcMotor.get("lw");
        rightSide = hardwareMap.dcMotor.get("rw");
        rightSide.setDirection(DcMotorSimple.Direction.REVERSE);

        //leftSide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        rightSide.setTargetPosition(targetValue);

        rightSide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        long t0 = System.currentTimeMillis();

        while(opModeIsActive()) {
            final long t1 = System.currentTimeMillis();
            final long dt = t1 - t0;
            t0 = t1;

            setPoint = rightSide.getCurrentPosition();

            error = targetValue - setPoint;

            integral += Ki * error * dt;

            u = (Kp * error + integral + Kd * (error-previousError) / dt);

            previousError = error;

            //leftSide.setPower(u);

            rightPower = u;

            rightSide.setPower(rightPower);

            telemetry.addData("Output", u);
            telemetry.addData("Power", rightSide.getPower());
            telemetry.addData("Error", error);
            telemetry.addData("previousError", previousError);
            telemetry.update();
        }
    }
}