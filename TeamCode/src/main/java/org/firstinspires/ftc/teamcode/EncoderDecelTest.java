package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@Autonomous(name="Encoder Decel test", group="Test")
public class EncoderDecelTest extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
       EncoderDrive(1, 50, 15, 35, Forward);
       EncoderDrive(1, 50, 15,35, Reverse);
    }
}