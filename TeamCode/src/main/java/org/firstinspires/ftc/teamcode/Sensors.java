package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name="Sensor teleop", group="TESTS")
public class Sensors extends OpMode {
    public ColorSensor BackDistance;
    public void init() {
        BackDistance = hardwareMap.get(ColorSensor .class, "ConveyorSensor");
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
        telemetry.addData("Alpha", BackDistance.alpha());
        telemetry.addData("R", BackDistance.red());
        telemetry.addData("G", BackDistance.green());
        telemetry.addData("B", BackDistance.blue());
        telemetry.addData("ARGB", BackDistance.argb());
        telemetry.update();
        // End Telemetry
    }
}