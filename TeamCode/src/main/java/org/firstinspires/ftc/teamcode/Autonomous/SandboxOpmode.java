package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Sandbox test", group="Test")
public class SandboxOpmode extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        double intakeValLeft = 16;
        vuforiaHardware.Init(hardwareMap);
        sleep(500);
        telemetry.addData("Vumark", CryptoKey);
        telemetry.update();
        sleep(5000);
    }
}