package com.qualcomm.ftcrobotcontroller.opmodes;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class AutoRampUp extends LinearOpMode {

    DcMotor LeftDrive1;
    DcMotor LeftDrive2;
    DcMotor RightDrive1;
    DcMotor RightDrive2;
    DcMotor Lift;
    DcMotor Sweeper;
    DcMotor Arm;
    Servo ScoopLeft;
    Servo ScoopRight;

    OpticalDistanceSensor opticalDistanceSensor;
    OpticalDistanceSensor opticalDistanceSensor2;
    ColorSensor colorSensor;
    GyroSensor sensorGyro;

    public double DegreesToTurn;
    int LeftTarget;
    int RightTarget;

    DeviceInterfaceModule cdim;

    // All the methods:

    // This is the Drive Method.
    // It will take in two static values: Distance, and Direction.
    // It will then take the distance value (in inches), convert it to encoder readings, and travel the distance.
    public void Drive(int Distance,int Direction) {

        double Inches = Distance * Direction; // Inches to drive
        int ENCODER_CPR = 1440; // Encoder counts per Rev
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


    // This is the Gyro Turn Method.
    // It will take in a value of degrees to turn and a direction value (positive for right, negative for left)
    // It will then us the Gyro Sensor to turn that many degrees in the correct direction
    public void GyroTurn(int Degrees, float Direction) {

        LeftDrive1.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        LeftDrive2.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        RightDrive1.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        RightDrive2.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        if (Direction < 0) {
            DegreesToTurn = 360 - Degrees;
        } else {
            DegreesToTurn = Degrees;
        }

        if (Direction > 0) {
            while (sensorGyro.getHeading() < DegreesToTurn) {
                LeftDrive1.setPower(0.5);
                LeftDrive2.setPower(0.5);
                RightDrive1.setPower(-0.5);
                RightDrive2.setPower(-0.5);
                telemetry.addData("Heading", sensorGyro.getHeading());
                telemetry.addData("Text", "TurningRight");
            }
            LeftDrive1.setPower(0);
            LeftDrive2.setPower(0);
            RightDrive1.setPower(0);
            RightDrive2.setPower(0);
            telemetry.addData("Text", "Turning point reached");
        } else {
            while (sensorGyro.getHeading() > DegreesToTurn) {
                LeftDrive1.setPower(-0.5);
                LeftDrive2.setPower(-0.5);
                RightDrive1.setPower(0.5);
                RightDrive2.setPower(0.5);
                telemetry.addData("Heading", sensorGyro.getHeading());
                telemetry.addData("Text", "TurningLeft");
            }
            LeftDrive1.setPower(0);
            LeftDrive2.setPower(0);
            RightDrive1.setPower(0);
            RightDrive2.setPower(0);
            telemetry.addData("Text", "Turning point reached");
        }
    }

    //This is the Line Tracking Method.
    // It will track along a white line until the specified time has elapsed.
    public void LineTrack(long time) {

        LeftDrive1.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        LeftDrive2.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        RightDrive1.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        RightDrive2.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        long Timer = 0;

        while (Timer < time) {

            Timer = Timer + 1;

            double reflectance = opticalDistanceSensor.getLightDetected();

            if (reflectance <= 0.60) {
                LeftDrive1.setPower(0.2);
                LeftDrive2.setPower(0.2);
                RightDrive1.setPower(0);
                RightDrive2.setPower(0);
            } else {
                LeftDrive1.setPower(0);
                LeftDrive2.setPower(0);
                RightDrive1.setPower(0.2);
                RightDrive2.setPower(0.2);
            }
        }
    }

    // This is the (Unfinished) Color Recognition Method.
    // It will detect the color of the beacon and return a true value after it pushes the correct button.
    // If it does not detect a color it will extend the lift 2 inches (since it is likely there is debris in the way)
    // It will then return a false value, causing the method to be called again.
    public void ColorRecognition() {

        colorSensor.enableLed(false);
        float hsvValues[] = {0, 0, 0};
        final float values[] = hsvValues;
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);

        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        telemetry.addData("Clear", colorSensor.alpha());
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
        telemetry.addData("Hue", hsvValues[0]);

        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
            }
        });
        if (colorSensor.red() >= 1) {
            // extend left button pusher
            telemetry.addData("Text", "Red Detected");
        } else if (colorSensor.blue() >= 1) {
            // extend right button pusher
            telemetry.addData("Text", "Blue Detected");
        } else {
            // extend lift two inches
            Lift.setPower(1);
            try {
                Thread.sleep(2000);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }
            Lift.setPower(0);

            telemetry.addData("Text", "None Detected");
            if (colorSensor.red() >= 1) {
                // extend left button pusher
                telemetry.addData("Text", "Red Detected");
            } else if (colorSensor.blue() >= 1) {
                // extend right button pusher
                telemetry.addData("Text", "Blue Detected");
            } else {
                // Maintain Servo Position
            }
        }
    }

    // This is the BreakBlock method. It sets the drive train to 0 power.
    public void BreakBlock() {
        LeftDrive1.setPower(0);
        LeftDrive2.setPower(0);
        RightDrive1.setPower(0);
        RightDrive2.setPower(0);
    }


    // AND HERE WE ARE!
    // Below is the actual program. :P


    public void runOpMode() throws InterruptedException {


        LeftDrive1 = hardwareMap.dcMotor.get("Motor1");
        LeftDrive2 = hardwareMap.dcMotor.get("Motor2");
        RightDrive1 = hardwareMap.dcMotor.get("Motor3");
        RightDrive2 = hardwareMap.dcMotor.get("Motor4");
        RightDrive1.setDirection(DcMotor.Direction.REVERSE);
        RightDrive2.setDirection(DcMotor.Direction.REVERSE);

        Lift = hardwareMap.dcMotor.get("Lift");
        Sweeper = hardwareMap.dcMotor.get("Sweeper");
        Arm = hardwareMap.dcMotor.get("Arm");

        ScoopLeft = hardwareMap.servo.get("ScoopLeft");
        ScoopRight = hardwareMap.servo.get("ScoopRight");
        waitForStart();

       LeftDrive1.setPower(1);
       LeftDrive2.setPower(1);
       sleep(1000);

       BreakBlock();
    }
}
