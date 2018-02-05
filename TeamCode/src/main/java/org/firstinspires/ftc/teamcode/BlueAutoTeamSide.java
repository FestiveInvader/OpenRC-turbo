package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="BLUE| Left Side regular", group="BLUE")
public class BlueAutoTeamSide extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        knockOffJewel("BLUE");
        EncoderDrive(.15, 24, Reverse);
        findWall(.2, 56);
        JewelArm.setPosition(JewelServoUpPos);
        gyroTurn(.225, 89);
        EncoderDrive(.2, 10, Forward);
        driveAndPlace(CryptoKey, Reverse, TeamSide, 0);
    }
}