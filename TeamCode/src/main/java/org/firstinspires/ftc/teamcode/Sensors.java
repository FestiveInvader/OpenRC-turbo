package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Sensor teleop", group="TESTS")
public class Sensors extends OpMode {
    public ModernRoboticsI2cRangeSensor BackDistance;
    public void init() {
        BackDistance = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "RightDistance");
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
        telemetry.addData("Intake Distance", BackDistance.getDistance(DistanceUnit.CM));
        telemetry.update();
        // End Telemetry
    }
}