package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="BLUE| Left Side regular", group="BLUE")
public class BlueAutoTeamSide extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        knockOffJewel("BLUE", 2);
        EncoderDrive(.15, 24, Reverse,stayOnHeading, 4);
        JewelArm.setPosition(JewelServoUpPos);
        gyroTurn(.225, -89);
        driveAndPlace(CryptoKey, Reverse, TeamSide, 0, 2);
        //ramThePitTeamSide(2, Reverse);
        endAuto();

    }
}