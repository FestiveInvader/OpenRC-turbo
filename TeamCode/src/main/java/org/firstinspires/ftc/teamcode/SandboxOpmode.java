package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.TimeUnit;

@Autonomous(name="Sandbox test", group="Test")
public class SandboxOpmode extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        gyroTurn(turningSpeed, 90);
        gyroTurn(turningSpeed, -90);
        gyroTurn(turningSpeed, 180);
        gyroTurn(turningSpeed, -180);
        gyroTurn(turningSpeed, 360);



    }

}