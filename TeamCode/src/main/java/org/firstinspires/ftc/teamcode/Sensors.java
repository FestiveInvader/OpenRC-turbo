package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Sensor teleop", group="TESTS")
public class Sensors extends OpMode {
    public I2CXLv2 BackDistance;
    public I2CXLv2 FrontDistance;
    public DistanceSensor FrontLeftDistance;
    public DistanceSensor FrontRightDistance;
    public ColorSensor IntakeColor;
    public DistanceSensor IntakeDistance;
    public void init() {
        BackDistance = hardwareMap.get(I2CXLv2.class, "BackDistance");
        FrontDistance = hardwareMap.get(I2CXLv2.class, "FrontDistance");
        IntakeColor = hardwareMap.get(ColorSensor.class, "IntakeSensor");
        IntakeDistance = hardwareMap.get(DistanceSensor.class, "IntakeSensor");
        FrontLeftDistance = hardwareMap.get(DistanceSensor.class, "FrontLeftSensor");
        FrontRightDistance = hardwareMap.get(DistanceSensor.class, "FrontRightSensor");
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
        telemetry.addData("Back Distance, ", BackDistance.getDistance());
        telemetry.addData("Front Distance, ", FrontDistance.getDistance());
        telemetry.addData("Intake Distance", IntakeDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("FrontLeft Distance", FrontLeftDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("FrontRight Distance", FrontRightDistance.getDistance(DistanceUnit.CM));
        if(IntakeColor.alpha() > 130){
            telemetry.addData("Grey", IntakeColor.alpha());
        }else{
            telemetry.addData("Brown", IntakeColor.alpha());
        }
        telemetry.addData("Intake Red", IntakeColor.red());
        telemetry.addData("Intake Green", IntakeColor.green());
        telemetry.addData("Intake Blue", IntakeColor.blue());
        telemetry.update();

        // End Telemetry
    }
}