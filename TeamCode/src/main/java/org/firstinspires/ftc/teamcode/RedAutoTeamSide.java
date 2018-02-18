package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name="Red| Team Side regular", group="RED")
public class RedAutoTeamSide extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        knockOffJewel("RED");
        EncoderDrive(.2,  24, Forward);
        gyroTurn(.215, -90);
        EncoderDrive(.2,  12, Reverse);
        driveAndPlace(CryptoKey, Forward, TeamSide, 0, 3);
        endAuto();


    }
}