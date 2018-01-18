package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name="Red| Relic Side regular", group="RED")
public class RedAutoRelicSide extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        knockOffJewel("RED");
        driveAndPlace(CryptoKey, Forward, RelicSide, 0);
    }
}