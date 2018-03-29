package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BLUE| Right Side MG", group="BLUE")
public class BlueAutoRelicSideMG extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        knockOffJewel("BLUE",1);
        driveAndPlace(CryptoKey, Reverse, RelicSide, 0,1);
        ramThePitRelicSide(1, Reverse);
        endAuto();
    }
}