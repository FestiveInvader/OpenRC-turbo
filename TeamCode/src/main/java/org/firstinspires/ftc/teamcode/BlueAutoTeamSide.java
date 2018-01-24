package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name="BLUE| Left Side regular", group="BLUE")
public class BlueAutoTeamSide extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        knockOffJewel("BLUE");
        findWall(-.2, 18);
        EncoderDrive(.25,  21,3,3, Forward);
        gyroTurn(.215, 89);
        EncoderDrive(.15,    4, 1,1, Forward);
        driveAndPlace(CryptoKey, Reverse, TeamSide, 0);
    }
}