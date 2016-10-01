package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


public class BasicTeleop extends OpMode{
    // This is an "Opmode".
    // A regular Opmode always needs to have a Loop in it - otherwise it won't work.
    // The other option is a "Linear OpMode".
    // It doesn't require the loop and is mostly used for autonomous programming.

    DcMotor LeftDrive1;
    DcMotor LeftDrive2;
    DcMotor RightDrive1;
    DcMotor RightDrive2;
    // "DcMotor" declares that this name is for a motor.
    // "LeftDrive" and "RightDrive" are names for the individual motors.

    float LeftPower;
    float RightPower;

    public void init(){
        // The "init" section is run when you hit the "init" button on the phone when starting your program.
        // It links all of your declared motors and servos to the ones on the robot, and will give you errors if they don't match up.

        LeftDrive1 = hardwareMap.dcMotor.get("LeftDrive1");
        LeftDrive2 = hardwareMap.dcMotor.get("LeftDrive2");
        RightDrive1 = hardwareMap.dcMotor.get("RightDrive1");
        RightDrive2 = hardwareMap.dcMotor.get("RightDrive2");
        LeftDrive1.setDirection(DcMotor.Direction.REVERSE);
        LeftDrive2.setDirection(DcMotor.Direction.REVERSE);
        // This section collects the motors you set up previously and links them to names in your config file on the robot.
    }
    public void loop(){
        // The loop section starts when you hit the play button on the phone after you have initialized it.
        // All your main tele-op code will go here.

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
        // This sets the power of each motor to the values of the game controllers.
        // Values for both the motor power and the joystick position range from 0 to 1.

    }
}
//Jabe was here, you have been sponsored.