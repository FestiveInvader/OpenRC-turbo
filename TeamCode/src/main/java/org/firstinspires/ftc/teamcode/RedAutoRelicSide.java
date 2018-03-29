package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red| Relic Side regular", group="RED")
public class RedAutoRelicSide extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        knockOffJewel("RED", 4);
        driveAndPlace(CryptoKey, Forward, RelicSide, 0, 4);
        //ramThePitRelicSide(4, Forward);
        endAuto();
    }
}