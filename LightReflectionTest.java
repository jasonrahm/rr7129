package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by Knut on 10/24/2015.
 */
public class LightReflectionTest extends LinearOpMode {

    //OpticalDistanceSensor opticalDistanceSensor;
    LightSensor lightSensor;

    public void runOpMode () throws InterruptedException {
        //opticalDistanceSensor = hardwareMap.opticalDistanceSensor.get ("OpticalDistanceSensor");
        //opticalDistanceSensor.enableLed(true);
        lightSensor = hardwareMap.lightSensor.get("lightSensor");
        lightSensor.enableLed(true);

        waitForStart();

        while (opModeIsActive()) {
            //double reflectance = opticalDistanceSensor.getLightDetectedRaw();
            double reflectance = lightSensor.getLightDetected();
            telemetry.addData("Reflectance", reflectance);
    }
    }
}
