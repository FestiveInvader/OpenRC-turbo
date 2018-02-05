package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name="GyroTurn Test", group="TESTS")
public class gyroTurnTest extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        gyroTurn(.3, 90);
        sleep(20000);
    }
}