package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="BLUE| Left Side regular", group="BLUE")
public class BlueAutoTeamSide extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        knockOffJewel("BLUE");
        EncoderDrive(.25, 24, Reverse);
        EncoderDrive(.25,  21,Forward);
        gyroTurn(.215, 89);
        EncoderDrive(.15,    4, Forward);
        driveAndPlace(CryptoKey, Reverse, TeamSide, 0);
    }
}