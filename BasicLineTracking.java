package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by Knut on 10/15/2015.
 */
public class BasicLineTracking extends OpMode {

    DcMotor LeftDrive1;
    DcMotor LeftDrive2;
    DcMotor RightDrive1;
    DcMotor RightDrive2;

    OpticalDistanceSensor opticalDistanceSensor;

    public BasicLineTracking() {
    }

    public void init() {

        LeftDrive1 = hardwareMap.dcMotor.get("Motor1");
        LeftDrive2 = hardwareMap.dcMotor.get("Motor2");
        RightDrive1 = hardwareMap.dcMotor.get("Motor3");
        RightDrive2 = hardwareMap.dcMotor.get("Motor4");
        RightDrive1.setDirection(DcMotor.Direction.REVERSE);
        RightDrive2.setDirection(DcMotor.Direction.REVERSE);

        opticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("OpticalDistanceSensor");

    }

    public void loop() {

        double reflectance = opticalDistanceSensor.getLightDetected();

        if (reflectance <= 0.60) {
            RightDrive1.setPower(0.2);
            RightDrive2.setPower(0.2);
            LeftDrive1.setPower(0);
            LeftDrive2.setPower(0);
        } else {
            LeftDrive1.setPower(0.2);
            LeftDrive2.setPower(0.2);
            RightDrive1.setPower(0);
            RightDrive2.setPower(0);
        }
    }


    public void stop() {

    }

}
