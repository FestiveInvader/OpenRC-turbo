package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name="BLUE| Right Side regular", group="BLUE")
public class BlueAutoRelicSide extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        knockOffJewel("BLUE");
        // EncoderDrive(.15, 12,12, Reverse);
        driveAndPlace(CryptoKey, Reverse, RelicSide, 0);
        //ramThePit();
        telemetry.addData("Cryptokey", CryptoKey);
        telemetry.update();
    }
}