package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.DeclarationsAutonomous;

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