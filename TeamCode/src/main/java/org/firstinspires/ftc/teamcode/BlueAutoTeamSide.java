package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="BLUE| Left Side regular", group="BLUE")
public class BlueAutoTeamSide extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        knockOffJewel("BLUE");
        EncoderDrive(.15, 28, Reverse);
        JewelArm.setPosition(JewelServoUpPos);
        gyroTurn(.225, 89);
        EncoderDrive(.2, 6, Forward);
        driveAndPlace(CryptoKey, Reverse, TeamSide, 0, 2);
        endAuto();

    }
}