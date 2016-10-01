package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.Range;

public class TMPTest extends LinearOpMode {

    DcMotor LeftDrive1;
    DcMotor LeftDrive2;
    DcMotor RightDrive1;
    DcMotor RightDrive2;
    int LeftTarget;
    int RightTarget;

    public void Drive (int Distance, int MaxSpeed, int Direction){
        double Inches = Distance * Direction; // Inches to drive
        int ENCODER_CPR = 1440; // Encoder counts per Rev
        double ROTATIONS = Inches / 12.5; // Number of rotations to drive
        double TARGET = ENCODER_CPR * ROTATIONS; // Number of encoder counts to drive

        LeftDrive1.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        LeftDrive2.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        RightDrive1.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        RightDrive2.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        LeftTarget = (LeftDrive1.getCurrentPosition() + (int) TARGET);
        RightTarget = (RightDrive1.getCurrentPosition() + (int) TARGET);

        double motorspeed= .2;
        motorspeed = Range.clip (motorspeed, -1, 1);
        float DistanceGone;

        //accelerate
        while (motorspeed < MaxSpeed && Math.abs(RightDrive1.getCurrentPosition()) < Math.abs (RightTarget))
        {
            motorspeed = motorspeed + .2;

            RightDrive1.setPower(motorspeed * Direction);
            RightDrive2.setPower(motorspeed*Direction);
            LeftDrive1.setPower(motorspeed * Direction);
            LeftDrive2.setPower(motorspeed * Direction);
            telemetry.addData ("Text", "Accelerating");
        }

        //max speed

            DistanceGone = Math.abs(RightDrive1.getCurrentPosition());
            motorspeed = MaxSpeed;

            while (Math.abs(RightDrive1.getCurrentPosition()) < Math.abs(RightDrive1.getTargetPosition()))
            {
                RightDrive1.setPower(motorspeed * Direction);
                RightDrive2.setPower(motorspeed * Direction);
                LeftDrive1.setPower(motorspeed * Direction);
                LeftDrive2.setPower(motorspeed * Direction);

                if (Math.abs (RightDrive1.getCurrentPosition()) < Math.abs (RightDrive1.getTargetPosition())&& Math.abs (RightDrive1.getCurrentPosition())
                        >= RightDrive1.getTargetPosition() -DistanceGone && motorspeed > .15)
                {
                    motorspeed = motorspeed-.2;
                    RightDrive1.setPower(motorspeed * Direction);
                    RightDrive2.setPower(motorspeed*Direction);
                    LeftDrive1.setPower(motorspeed * Direction);
                    LeftDrive2.setPower(motorspeed * Direction);
                    telemetry.addData("Text", "Driving");
                }
            }

        RightDrive1.setPower(0);
        RightDrive2.setPower(0);
        LeftDrive1.setPower(0);
        LeftDrive2.setPower(0);
        }

    public void runOpMode() throws InterruptedException {

        LeftDrive1 = hardwareMap.dcMotor.get("Motor1");
        LeftDrive2 = hardwareMap.dcMotor.get("Motor2");
        RightDrive1 = hardwareMap.dcMotor.get("Motor3");
        RightDrive2 = hardwareMap.dcMotor.get("Motor4");
        RightDrive1.setDirection(DcMotor.Direction.REVERSE);
        RightDrive2.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        Drive(50,1,1);
    }
}