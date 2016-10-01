package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class TeleopSabertooth extends OpMode{


    DcMotor LeftDrive;
    DcMotor RightDrive;
    DcMotor Lift1;
    DcMotor Lift2;
    DcMotor Lift3;
    DcMotor Lift4;
    DcMotor Collection;
    DcMotor Hopper;
    Servo Fang;

    //Servo FangLeft;
    //Servo FangRight;
    //Servo ClimberRight;
    //Servo ClimberLeft;

    double LeftFang;
    double RightFang;
    double RightClimber;
    double LeftClimber;

    float LeftPower;
    float RightPower;

    public void init() {

        LeftDrive = hardwareMap.dcMotor.get ("LeftDrive");
        RightDrive = hardwareMap.dcMotor.get ("RightDrive");
        Collection = hardwareMap.dcMotor.get("Collection");
        Lift1 = hardwareMap.dcMotor.get("Lift1");
        Lift2 = hardwareMap.dcMotor.get("Lift2");
        Lift3 = hardwareMap.dcMotor.get("Lift3");
        Lift4 = hardwareMap.dcMotor.get("Lift4");
        Hopper = hardwareMap.dcMotor.get("Hopper");
        Fang = hardwareMap.servo.get("Fang");

        //ClimberLeft = hardwareMap.servo.get("ClimberLeft");
        //ClimberRight = hardwareMap.servo.get("ClimberRight");
        //FangLeft = hardwareMap.servo.get("FangLeft");
        //FangRight = hardwareMap.servo.get("FangRight");

        RightDrive.setDirection (DcMotor.Direction.REVERSE);

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

            RightPower = gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y);
            LeftPower = -gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y);
        LeftDrive.setPower(LeftPower);
        RightDrive.setPower(RightPower);

        if (gamepad1.left_bumper){
            LeftFang = (0);
            RightFang = (1);
        }
        Fang.setPosition(LeftFang);
        //FangLeft.setPosition(LeftFang);
        //FangRight.setPosition(RightFang);


        if (gamepad1.right_bumper){
            LeftFang = (.5);
            RightFang = (0);
        }
        //FangLeft.setPosition (LeftFang);
        //FangRight.setPosition(RightFang);

        if (gamepad2.x){
            RightClimber = (.7);
        }

        //ClimberLeft.setPosition(LeftClimber);

        if (gamepad2.b){
            LeftClimber = (0.1);
        }
        //ClimberRight.setPosition(RightClimber);

        if (gamepad2.y){
            LeftClimber = .5;
            RightClimber = .4;
        }
        //ClimberLeft.setPosition(LeftClimber);
        //ClimberRight.setPosition(RightClimber);

        if (gamepad2.dpad_left){
            Hopper.setPower(-.1);
        }
        else if (gamepad2.dpad_right){
            Hopper.setPower(1);
        }
        else {
            Hopper.setPower(0);
        }
        // The sweeper will run based on the right joystick position
        Collection.setPower(gamepad2.right_stick_y);

        // If the top button of the dpad is pressed, then the lift will run upwards
        if (gamepad2.dpad_up){
            Lift1.setPower (1);
            Lift2.setPower (1);
            Lift3.setPower (-1);
            Lift4.setPower (-1);
        }
        // If the bottom button of the dpad is pressed, then the lift will run downwards
        else if (gamepad2.dpad_down){
            Lift1.setPower (-1);
            Lift2.setPower (-1);
            Lift3.setPower (1);
            Lift4.setPower (1);
        }
        // If no button is pressed the lift will do nothing
        else {
            Lift1.setPower (0);
            Lift2.setPower (0);
            Lift3.setPower (0);
            Lift4.setPower (0);
        }

    }
}
