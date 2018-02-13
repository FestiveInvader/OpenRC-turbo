package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Encoder Decel test", group="Test")
public class EncoderDecelTest extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
       EncoderDrive(.5, 100,  Forward);
       EncoderDrive(.5, 100, Reverse);
    }
}