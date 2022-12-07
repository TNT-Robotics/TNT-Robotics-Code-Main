package org.firstinspires.ftc.teamcode.driver;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class tryColor extends LinearOpMode {

    ColorRangeSensor color;
    private DistanceSensor sensorRange;
    @Override
    public void runOpMode() throws InterruptedException {

        color = hardwareMap.get(ColorRangeSensor.class, "claw");
        sensorRange = hardwareMap.get(DistanceSensor.class, "chassis");
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {

            telemetry.addData("Red", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());
            telemetry.addData("Distance", color.getDistance(DistanceUnit.CM));

            telemetry.addLine("Chassis");
            telemetry.addData("Distance", sensorRange.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
