package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Knut on 1/29/2016.
 */
public class ArmAutonomous extends LinearOpMode {
    DcMotor LeftDrive1;
    DcMotor LeftDrive2;
    DcMotor RightDrive1;
    DcMotor RightDrive2;
    Servo ClimberRight;
    Servo ClimberLeft;
    Servo Delivery;
    double RightClimber;
    double LeftClimber;

    public void runOpMode() throws InterruptedException {
        LeftDrive1 = hardwareMap.dcMotor.get("Motor1");
        LeftDrive2 = hardwareMap.dcMotor.get("Motor2");
        RightDrive1 = hardwareMap.dcMotor.get("Motor3");
        RightDrive2 = hardwareMap.dcMotor.get("Motor4");
        ClimberLeft = hardwareMap.servo.get("LeftClimber");
        ClimberRight = hardwareMap.servo.get("RightClimber");
        Delivery = hardwareMap.servo.get("Delivery2");

        LeftClimber = (.1);
        RightClimber = (.9);

        waitForStart();

        LeftDrive1.setPower(1);
        LeftDrive2.setPower(1);
        RightDrive1.setPower(1);
        RightDrive2.setPower(1);
        sleep(1000);

        LeftDrive1.setPower(1);
        LeftDrive2.setPower(1);
        RightDrive1.setPower(-1);
        RightDrive2.setPower(-1);
        sleep(100);

        LeftDrive1.setPower(1);
        LeftDrive2.setPower(1);
        RightDrive1.setPower(1);
        RightDrive2.setPower(1);
        sleep(500);

        LeftDrive1.setPower(1);
        LeftDrive2.setPower(1);
        RightDrive1.setPower(-1);
        RightDrive2.setPower(-1);
        sleep(100);

        LeftDrive1.setPower(1);
        LeftDrive2.setPower(1);
        RightDrive1.setPower(1);
        RightDrive2.setPower(1);
        sleep(1000);

    }
}
