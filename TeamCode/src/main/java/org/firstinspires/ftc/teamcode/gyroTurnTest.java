package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BLUE| Left Side regular", group="BLUE")
public class gyroTurnTest extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        gyroTurn(.25, 90);
        sleep(20000);
    }
}