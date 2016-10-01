package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;


public class GyroTurn extends LinearOpMode {


    DcMotor LeftDrive1;
    DcMotor LeftDrive2;
    DcMotor RightDrive1;
    DcMotor RightDrive2;

    GyroSensor sensorGyro;

    public double DegreesToTurn;

    DeviceInterfaceModule cdim;

    // This is the Gyro Turn Method.
    // It will take in a value of degrees to turn and a direction value (positive for right, negative for left)
    // It will then use the Gyro Sensor to turn that many degrees in the correct direction
    public void Turn(int Degrees, float Direction) {

        sensorGyro.calibrate();

        if (Direction < 0) {
            DegreesToTurn = 360 - Degrees;
        } else {
            DegreesToTurn = Degrees;
        }

        if (Direction > 0) {
            while (sensorGyro.getHeading() < DegreesToTurn) {
                LeftDrive1.setPower(0.3);
                LeftDrive2.setPower(0.3);
                RightDrive1.setPower(-0.3);
                RightDrive2.setPower(-0.3);
                telemetry.addData("Heading", sensorGyro.getHeading());
                telemetry.addData("Text", "TurningRight");
            }
            LeftDrive1.setPower(0);
            LeftDrive2.setPower(0);
            RightDrive1.setPower(0);

            RightDrive2.setPower(0);
            telemetry.addData("Text","Turning point reached");
        }
        else {
            while (sensorGyro.getHeading() > DegreesToTurn) {
                LeftDrive1.setPower(-0.3);
                LeftDrive2.setPower(-0.3);
                RightDrive1.setPower(0.3);
                RightDrive2.setPower(0.3);
                telemetry.addData("Heading", sensorGyro.getHeading());
                telemetry.addData("Text", "TurningLeft");
            }
            LeftDrive1.setPower(0);
            LeftDrive2.setPower(0);
            RightDrive1.setPower(0);
            RightDrive2.setPower(0);
            telemetry.addData("Text","Turning point reached");
        }
    }

    public void runOpMode() throws InterruptedException {


        LeftDrive1 = hardwareMap.dcMotor.get("Motor1");
        LeftDrive2 = hardwareMap.dcMotor.get("Motor2");
        RightDrive1 = hardwareMap.dcMotor.get("Motor3");
        RightDrive2 = hardwareMap.dcMotor.get("Motor4");
        RightDrive1.setDirection(DcMotor.Direction.REVERSE);
        RightDrive2.setDirection(DcMotor.Direction.REVERSE);

        hardwareMap.logDevices();

        sensorGyro = hardwareMap.gyroSensor.get("gyro");
        cdim = hardwareMap.deviceInterfaceModule.get ("Device Interface Module 1");

        waitForStart();

        Turn (90,1);

    }
}
