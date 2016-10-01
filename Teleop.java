package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Teleop extends OpMode{


    DcMotor LeftDrive1;
    DcMotor LeftDrive2;
    DcMotor RightDrive1;
    DcMotor RightDrive2;
    DcMotor Delivery;
    DcMotor Sweeper;
    DcMotor Arm;
    DcMotor Lift;

    Servo FangLeft;
    Servo FangRight;
    Servo ClimberRight;
    Servo ClimberLeft;

    double LeftFang;
    double RightFang;
    double RightClimber;
    double LeftClimber;

    float LeftPower;
    float RightPower;

    public void init() {

        LeftDrive1 = hardwareMap.dcMotor.get ("Motor1");
        LeftDrive2 = hardwareMap.dcMotor.get ("Motor2");
        RightDrive1 = hardwareMap.dcMotor.get ("Motor3");
        RightDrive2 = hardwareMap.dcMotor.get ("Motor4");
        Sweeper = hardwareMap.dcMotor.get("Sweeper");
        Arm = hardwareMap.dcMotor.get("Arm");
        Lift = hardwareMap.dcMotor.get("Lift");
        Delivery = hardwareMap.dcMotor.get("Delivery");
        ClimberLeft = hardwareMap.servo.get("ClimberLeft");
        ClimberRight = hardwareMap.servo.get("ClimberRight");
        FangLeft = hardwareMap.servo.get("FangLeft");
        FangRight = hardwareMap.servo.get("FangRight");

        RightDrive1.setDirection (DcMotor.Direction.REVERSE);
        RightDrive2.setDirection (DcMotor.Direction.REVERSE);

        LeftFang = (.1);
        RightFang = (.9);
        LeftClimber = (.5);
        RightClimber = (.4);
    }
    public void loop(){

        // This is the exponential driver switchcase
        // It takes the joystick inputs and multiplies them by their absolute value, giving a controlled cubing function

        // If no button is pressed, driving is standard
        // While the joystick buttons are pressed the drive controls enter "Climbing Mode".
        // This reverses which direction of the robot is front, and divides the motor power by two
        if (gamepad1.right_stick_button){
            RightPower = -gamepad1.left_stick_y * Math.abs(gamepad1.right_stick_y)/2;
        }
        else {
            RightPower = gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y);

        }
        if (gamepad1.left_stick_button){
            LeftPower = -gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)/2;
        }
        else {
            LeftPower = gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y);
        }
        LeftDrive1.setPower(LeftPower);
        LeftDrive2.setPower(LeftPower);
        RightDrive1.setPower(RightPower);
        RightDrive2.setPower(RightPower);

        if (gamepad1.left_bumper){
            LeftFang = (0);
            RightFang = (1);
        }
        FangLeft.setPosition(LeftFang);
        FangRight.setPosition(RightFang);


        if (gamepad1.right_bumper){
            LeftFang = (1);
            RightFang = (0);
        }
        FangLeft.setPosition (LeftFang);
        FangRight.setPosition(RightFang);

        if (gamepad2.x){
            RightClimber = (.7);
        }

        ClimberLeft.setPosition(LeftClimber);

        if (gamepad2.b){
            LeftClimber = (0.1);
        }
        ClimberRight.setPosition(RightClimber);

        if (gamepad2.y){
            LeftClimber = .5;
            RightClimber = .4;
        }
        ClimberLeft.setPosition(LeftClimber);
        ClimberRight.setPosition(RightClimber);

        if (gamepad2.dpad_left){
            Delivery.setPower(-.75);
        }
        else if (gamepad2.dpad_right){
            Delivery.setPower(.75);
        }
        else {
            Delivery.setPower(0);
        }
        // The sweeper will run based on the right joystick position
        Sweeper.setPower(gamepad2.left_stick_y * -1);

        // The arm will be powered based on the left joystick position
        Arm.setPower(gamepad2.right_stick_y);

        // If the top button of the dpad is pressed, then the lift will run upwards
        if (gamepad2.dpad_up){
            Lift.setPower (1);
        }
        // If the bottom button of the dpad is pressed, then the lift will run downwards
        else if (gamepad2.dpad_down){
            Lift.setPower (-1);
        }
        // If no button is pressed the lift will do nothing
        else {
            Lift.setPower (0);
        }

    }
    public void stop() {
        //Nothing here either
    }
}
