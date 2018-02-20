package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Encoder Decel test", group="Test")
public class EncoderDecelTest extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        ramThePitTeamSide(3, Forward);
        endAuto();
    }
}