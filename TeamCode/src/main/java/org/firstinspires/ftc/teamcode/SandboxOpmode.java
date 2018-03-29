package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Sandbox test", group="Test")
public class SandboxOpmode extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        double timer;
        boolean wentLeft = false;
        boolean wentRight = false;
        boolean gotGlyph = false;
        ConveyorRight.setPower(1);
        ConveyorLeft.setPower(1);
        boolean linedUp = false;
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double startingEncoderCount = FrontLeft.getCurrentPosition();
        double limitEncoderCount = startingEncoderCount + 18*CountsPerInch;

        while(runtime.seconds() < 25 && !gotGlyph && opModeIsActive() && (Math.abs(FrontLeft.getCurrentPosition()) < Math.abs(limitEncoderCount)) ) {
            moveBy(.15, 0, 0);
            if (FlipperDistance2.getDistance(DistanceUnit.CM) < 75) {
                gotGlyph = true;
            }
        }
        stopDriveMotors();
        sleep(5000);
    }
}