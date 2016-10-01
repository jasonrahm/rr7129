package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

    public class EncoderTest extends LinearOpMode {
        DcMotor RightDrive1;
        DcMotor RightDrive2;
        DcMotor LeftDrive1;
        DcMotor LeftDrive2;
        int LeftTarget;
        int RightTarget;

    // This is the Drive Method.
    // It will take in two static values: Distance, and Direction.
    // It will then take the distance value (in inches), convert it to encoder readings, and travel the distance.
    public void Drive(int Distance,int Direction) {

        double Inches = Distance * Direction; // Inches to drive
        int ENCODER_CPR = 1120; // Encoder counts per Rev
        double ROTATIONS = Inches / 12.5; // Number of rotations to drive
        double TARGET = ENCODER_CPR * ROTATIONS; // Number of encoder counts to drive

        LeftDrive1.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        LeftDrive2.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        RightDrive1.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        RightDrive2.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        LeftTarget = ((LeftDrive1.getCurrentPosition() + (int) TARGET)*Direction);
        RightTarget = ((RightDrive1.getCurrentPosition() + (int) TARGET)*Direction);

        if (Direction > 0) {
            while ((Math.abs(LeftDrive1.getCurrentPosition()) <= (Math.abs(LeftTarget) - 5)) &&
                    (Math.abs(RightDrive1.getCurrentPosition()) <= (Math.abs(RightTarget) - 5))) {
                LeftDrive1.setPower(.8);
                LeftDrive2.setPower(.8);
                RightDrive1.setPower(.8);
                RightDrive2.setPower(.8);
                telemetry.addData("Text", "Driving Forward");
            }
            LeftDrive1.setPower(0);
            LeftDrive2.setPower(0);
            RightDrive1.setPower(0);
            RightDrive2.setPower(0);
            telemetry.addData("Text", "Stopped");
        }
        else {
            while ((Math.abs(LeftDrive1.getCurrentPosition()) <= (Math.abs(LeftTarget) - 5)) &&
                    (Math.abs(RightDrive1.getCurrentPosition()) <= (Math.abs(RightTarget) - 5))) {
                LeftDrive1.setPower(-.8);
                LeftDrive2.setPower(-.8);
                RightDrive1.setPower(-.8);
                RightDrive2.setPower(-.8);
                telemetry.addData("Text", "Driving Backward");
            }
            LeftDrive1.setPower(0);
            LeftDrive2.setPower(0);
            RightDrive1.setPower(0);
            RightDrive2.setPower(0);
            telemetry.addData("Text", "Stopped");
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        LeftDrive1 = hardwareMap.dcMotor.get("Motor1");
        LeftDrive2 = hardwareMap.dcMotor.get("Motor2");
        RightDrive1 = hardwareMap.dcMotor.get("Motor3");
        RightDrive2 = hardwareMap.dcMotor.get("Motor4");

        RightDrive1.setDirection(DcMotor.Direction.REVERSE);
        RightDrive2.setDirection(DcMotor.Direction.REVERSE);

        LeftDrive1.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        LeftDrive2.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        RightDrive1.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        RightDrive2.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        waitOneFullHardwareCycle();

        while(opModeIsActive()){
            telemetry.addData("RightPosition", RightDrive1.getCurrentPosition());
            telemetry.addData("LeftPosition", LeftDrive1.getCurrentPosition());
        }
        waitForStart();

        Drive(40,1);

    }
}
