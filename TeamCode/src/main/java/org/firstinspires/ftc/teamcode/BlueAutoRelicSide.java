package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name="BLUE| Right Side regular", group="BLUE")
public class BlueAutoRelicSide extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        knockOffJewel("BLUE");
        driveAndPlace(CryptoKey, Reverse, RelicSide, 0);
        ramThePit();
        endAuto();
    }
}