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

public class BlueAutoBen extends LinearOpMode {

    public double DegreesToTurn;
    DcMotor LeftDrive1;
    DcMotor LeftDrive2;
    DcMotor RightDrive1;
    DcMotor RightDrive2;
    DcMotor Sweeper;
    DcMotor Delivery;
    DcMotor Arm;
    DcMotor Lift;
    Servo LBB;
    // Left Beacon Button
    Servo RBB;
    // Right Beacon Button
    Servo ScoopLeft;
    Servo ScoopRight;
    OpticalDistanceSensor opticalDistanceSensor;
    OpticalDistanceSensor opticalDistanceSensor2;
    ColorSensor colorSensor;
    GyroSensor sensorGyro;
    DeviceInterfaceModule cdim;

    // All the methods:

    // This is the Trapezoidal Move Profile (AKA Drive Method)
    // It will take in three static values: Distance, max speed, and direction.
    // It will then use those values to travel the distance, accelerating up to the max speed for the first third of the distance,
    // maintaining that speed the next third, and then decelerating to a stop the last third.
    public void Drive(double Distance, double Direction) {

        double Inches = Distance * Direction;

        int ENCODER_CPR = 1440; // Encoder counts per Rev
        double ROTATIONS = Inches / 8; // Number of rotations to drive
        double COUNTS = ENCODER_CPR * ROTATIONS; // Number of encoder counts to drive


        LeftDrive1.setTargetPosition(LeftDrive1.getCurrentPosition() + (int) COUNTS);
        LeftDrive2.setTargetPosition(LeftDrive2.getCurrentPosition() + (int) COUNTS);
        RightDrive1.setTargetPosition(RightDrive1.getCurrentPosition() + (int) COUNTS);
        RightDrive2.setTargetPosition(RightDrive2.getCurrentPosition() + (int) COUNTS);

        LeftDrive1.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        LeftDrive2.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        RightDrive1.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        RightDrive2.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        if (Direction > 0) {
            LeftDrive1.setPower(0.8);
            LeftDrive2.setPower(0.8);
            RightDrive1.setPower(0.8);
            RightDrive2.setPower(0.8);
        } else {
            LeftDrive1.setPower(-0.8);
            LeftDrive2.setPower(-0.8);
            RightDrive1.setPower(-0.8);
            RightDrive2.setPower(-0.8);
        }
    }

    // This is the Gyro Turn Method.
    // It will take in a value of degrees to turn and a direction value (positive for right, negative for left)
    // It will then us the Gyro Sensor to turn that many degrees in the correct direction
    public void GyroTurn(int Degrees, float Direction) {

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
            telemetry.addData("Text", "Turning point reached");
        } else {
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
            telemetry.addData("Text", "Turning point reached");
        }
    }

    //This is the Line Tracking Method.
    // It will track along a white line until the back OD Sensor reads less than __ inches.
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


    // This is the Color Recognition Method.
    // It will detect the color of the beacon and return a true value after it pushes the correct button.
    // If it does not detect a color it will extend the lift 2 inches (since it is likely there is debris in the way)
    // It will then return a false value, causing the method to be called again.
    public int ColorRecognition() {
        int colorValue = 0;
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
            colorValue = 1;
            telemetry.addData("Text", "Red Detected");

        } else if (colorSensor.blue() >= 1) {
            // extend right button pusher
            colorValue = 2;
            telemetry.addData("Text", "Blue Detected");
        }
        return colorValue;

    }

    // This is the BreakBlock method. It sets the drivetrain to 0 power.
    public void BreakBlock(double Power) {

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

        Sweeper = hardwareMap.dcMotor.get("Sweeper");
        Delivery = hardwareMap.dcMotor.get("Delivery");
        Arm = hardwareMap.dcMotor.get("Arm");
        Lift = hardwareMap.dcMotor.get("Lift");
        LBB = hardwareMap.servo.get("LBB");
        RBB = hardwareMap.servo.get("RBB");
        ScoopLeft = hardwareMap.servo.get("ScoopLeft");
        ScoopRight = hardwareMap.servo.get("ScoopRight");

        hardwareMap.logDevices();
        opticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("OpticalDistanceSensor");
        opticalDistanceSensor2 = hardwareMap.opticalDistanceSensor.get("OpticalDistanceSensor2");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        sensorGyro = hardwareMap.gyroSensor.get("gyro");
        cdim = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");

        sensorGyro.calibrate();

        while (opModeIsActive()) {
            telemetry.addData("Heading", sensorGyro.getHeading());
        }

        waitForStart();

        //10inches=18inches//

        while (sensorGyro.isCalibrating()) {
            Thread.sleep(10);
        }

        Drive(50, 1);
    }
}
