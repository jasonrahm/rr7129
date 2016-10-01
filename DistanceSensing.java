package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by Knut on 11/1/2015.
 */
public class DistanceSensing extends LinearOpMode {

    OpticalDistanceSensor opticalDistanceSensor;

    public void runOpMode () throws InterruptedException {
        opticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("opticalDistanceSensor");

        waitForStart();

        while (opModeIsActive()) {
            double distance = opticalDistanceSensor.getLightDetected();
            telemetry.addData ("distance", distance);
        }
        }
}
