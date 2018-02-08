package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Sensor teleop", group="TESTS")
public class Sensors extends OpMode {
    public I2CXLv2 RightDistance;
    public I2CXLv2 BackDistance;
    public DistanceSensor IntakeDistance;
    public void init() {
        RightDistance = hardwareMap.get(I2CXLv2.class, "RightDistance");
        BackDistance = hardwareMap.get(I2CXLv2.class, "BackDistance");
        IntakeDistance = hardwareMap.get(DistanceSensor.class, "IntakeSensor");
        telemetry.addData("Done with init", 1);
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        //This resets the time to the start time, which may or may not have been a large discrepancy
    }

    @Override
    public void loop() {
        // Start Telemetry
        telemetry.addData("Right Distance, ", RightDistance.getDistance());
        telemetry.addData("Back Distance, ", BackDistance.getDistance());
        telemetry.addData("Intake Distance", IntakeDistance.getDistance(DistanceUnit.CM));
        telemetry.update();

        // End Telemetry
    }
}