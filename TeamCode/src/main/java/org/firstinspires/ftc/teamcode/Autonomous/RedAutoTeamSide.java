package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red| Team Side regular", group="RED")
public class RedAutoTeamSide extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        knockOffJewel("RED", 3);
        EncoderDrive(.2,  24, 4,4,Forward, stayOnHeading, 5);
        gyroTurn(.215, -90);
        driveAndPlace(CryptoKey, Forward, TeamSide, 0, 3);
        //ramThePitTeamSide(3, Forward);
        endAuto();


    }
}