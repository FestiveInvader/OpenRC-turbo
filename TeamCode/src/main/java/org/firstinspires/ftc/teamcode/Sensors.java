package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Sensor teleop", group="TESTS")
public class Sensors extends OpMode {
    public I2CXLv2 BackDistance;
    public void init() {
        BackDistance = hardwareMap.get(I2CXLv2.class, "BackDistance");
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
        telemetry.addData("Back Distance: ", BackDistance.getDistance());
        telemetry.update();
        // End Telemetry
    }
}