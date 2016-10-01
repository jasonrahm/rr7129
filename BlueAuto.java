package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;

public class BlueAuto extends LinearOpMode {

    DcMotor LeftDrive1;
    DcMotor LeftDrive2;
    DcMotor RightDrive1;
    DcMotor RightDrive2;
    DcMotor Sweeper;
    DcMotor Arm;
    DcMotor Lift;
    DcMotor Delivery;

    Servo ClimberRight;
    Servo ClimberLeft;
    Servo ClimberDelivery;

    ColorSensor colorSensor;
    DeviceInterfaceModule cdim;
    public enum ColorSensorDevice {ADAFRUIT, HITECHNIC_NXT, MODERN_ROBOTICS_I2C};
    public ColorSensorDevice device = ColorSensorDevice.MODERN_ROBOTICS_I2C;

    double RightClimber;
    double LeftClimber;
    double DeliveryClimber;
    // All the methods:

    // This is the Trapezoidal Move Profile (AKA Drive Method)
    // It will take in three static values: Distance, Max Speed, and Direction.
    // It will then use those values to travel the distance, accelerating up to the max speed for the first third of the distance,
    // maintaining that speed the next third, and then decelerating to a stop the last third to reduce impact shock and wheel slippage
    // This is the Trapezoidal Move Profile (AKA Drive Method)
    // It will take in three static values: Distance, max speed, and direction.
    // It will then use those values to travel the distance, accelerating up to the max speed for the first third of the distance,
    // maintaining that speed the next third, and then decelerating to a stop the last third.
    public void Drive(double Distance, double Direction) {
        double Inches = Distance * Direction;
        double GEAR_RATIO = 2;
        int ENCODER_CPR = 1120; // Encoder counts per Rev
        double WHEEL_DIAMETER = 5.5; //Diameter of the wheel in inches
        double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        double ROTATIONS = Inches / CIRCUMFERENCE; // Number of rotations to 0drive
        double COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO; // Number of encoder counts to drive

        LeftDrive1.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        LeftDrive2.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        RightDrive1.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        RightDrive2.setMode(DcMotorController.RunMode.RESET_ENCODERS);


        while (COUNTS >= LeftDrive1.getCurrentPosition()) {

            LeftDrive1.setTargetPosition(LeftDrive1.getCurrentPosition() + (int) COUNTS);
            LeftDrive2.setTargetPosition(LeftDrive2.getCurrentPosition() + (int) COUNTS);
            RightDrive1.setTargetPosition(RightDrive1.getCurrentPosition() + (int) COUNTS);
            RightDrive2.setTargetPosition(RightDrive2.getCurrentPosition() + (int) COUNTS);

            LeftDrive1.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            LeftDrive2.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            RightDrive1.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            RightDrive2.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

            if (Direction == 1) {
                telemetry.addData("Counts", COUNTS);
                telemetry.addData("DriveLeft", LeftDrive1.getCurrentPosition());
                telemetry.addData("DriveRight", RightDrive1.getCurrentPosition());
                telemetry.addData("Direction", 1);
                LeftDrive1.setPower(1);
                LeftDrive2.setPower(1);
                RightDrive1.setPower(1);
                RightDrive2.setPower(1);
            }
            if (Direction == -1)
                telemetry.addData("Counts", COUNTS);
            telemetry.addData("DriveLeft", LeftDrive1.getCurrentPosition());
            telemetry.addData("DriveRight", RightDrive1.getCurrentPosition());
            telemetry.addData("Direction", -1);
            LeftDrive1.setPower(-1);
            LeftDrive2.setPower(-1);
            RightDrive1.setPower(-1);
            RightDrive2.setPower(-1);
        }

        LeftDrive1.setPower(0);
        LeftDrive2.setPower(0);
        RightDrive1.setPower(0);
        RightDrive2.setPower(0);

    }

    // This is the Color Recognition Method.
    // It will detect the color of the beacon and return a true value after it pushes the correct button.
    // If it does not detect a color it will extend the lift 2 inches (since it is likely there is debris in the way)
    // It will then return a false value, causing the method to be called again.
    public void LineStop(long Time) {

        LeftDrive1.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        LeftDrive2.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        RightDrive1.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        RightDrive2.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        colorSensor.enableLed(true);
        while((colorSensor.green() <= 30)){
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            LeftDrive1.setPower(.4);
            LeftDrive2.setPower(.4);
            RightDrive1.setPower(.4);
            RightDrive2.setPower(.4);
        }
        LeftDrive1.setPower(0);
        LeftDrive2.setPower(0);
        RightDrive1.setPower(0);
        RightDrive2.setPower(0);
    }



    public void BreakBlock(double Power) {
        LeftDrive1.setPower(Power);
        LeftDrive2.setPower(Power);
        RightDrive1.setPower(Power);
        RightDrive2.setPower(Power);
    }


    public void runOpMode() throws InterruptedException {

        LeftDrive1 = hardwareMap.dcMotor.get("Motor1");
        LeftDrive2 = hardwareMap.dcMotor.get("Motor2");
        RightDrive1 = hardwareMap.dcMotor.get("Motor3");
        RightDrive2 = hardwareMap.dcMotor.get("Motor4");
        Sweeper = hardwareMap.dcMotor.get("Sweeper");
        Arm = hardwareMap.dcMotor.get("Arm");
        Lift = hardwareMap.dcMotor.get("Lift");
        ClimberLeft = hardwareMap.servo.get("ClimberLeft");
        ClimberRight = hardwareMap.servo.get("ClimberRight");
        colorSensor = hardwareMap.colorSensor.get("Color");

        RightDrive1.setDirection(DcMotor.Direction.REVERSE);
        RightDrive2.setDirection(DcMotor.Direction.REVERSE);

        LeftClimber = (.1);
        RightClimber = (0);
        DeliveryClimber = (0);

        waitForStart();

        Drive(101, 1);
        BreakBlock(0);
        sleep(500);

        LeftDrive1.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        LeftDrive2.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        RightDrive1.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        RightDrive2.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        LeftDrive1.setPower(1);
        LeftDrive2.setPower(1);
        RightDrive1.setPower(-.2);
        RightDrive2.setPower(-.2);
        sleep(2750);
        BreakBlock(0);

        LineStop(1000);
        BreakBlock(0);

        LeftDrive1.setPower(.8);
        LeftDrive2.setPower(.8);
        RightDrive1.setPower(-.8);
        RightDrive2.setPower(-.8);
        sleep(1000);
        BreakBlock(0);

        LeftDrive1.setPower(-1);
        LeftDrive2.setPower(-1);
        RightDrive1.setPower(-1);
        RightDrive2.setPower(-1);
        sleep(1000);
        BreakBlock(0);

        DeliveryClimber=(.5);
        ClimberDelivery.setPosition(DeliveryClimber);

    }
}