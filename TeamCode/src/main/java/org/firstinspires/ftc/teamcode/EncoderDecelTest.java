package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Encoder Decel test", group="Test")
public class EncoderDecelTest extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
       EncoderDriveWAccecAndDecel(1, 50, 15, 35, Forward);
       EncoderDriveWAccecAndDecel(1, 50, 15,35, Reverse);
    }
}