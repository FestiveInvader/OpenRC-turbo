package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BLUE| Right Side regular", group="BLUE")
public class EncoderDecelTest extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
       EncoderDriveWAccecAndDecel(.5, 50, 4, 4, Forward);
       EncoderDriveWAccecAndDecel(.5, 50, 15,15, Reverse);
    }
}