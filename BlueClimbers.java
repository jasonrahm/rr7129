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

public class BlueClimbers extends LinearOpMode {

    DcMotor LeftDrive1;
    DcMotor LeftDrive2;
    DcMotor RightDrive1;
    DcMotor RightDrive2;
    DcMotor Lift;
    DcMotor Sweeper;
    DcMotor Arm;
    DcMotor Delivery;

    Servo ScoopLeft;
    Servo ScoopRight;
    Servo ClimberLeft;
    Servo ClimberRight;

    OpticalDistanceSensor opticalDistanceSensor;
    OpticalDistanceSensor opticalDistanceSensor2;
    ColorSensor colorSensor;
    GyroSensor sensorGyro;

    public double DegreesToTurn;

    double LeftClimber;
    double RightClimber;

    DeviceInterfaceModule cdim;

    // All the methods:

    // This is the Trapezoidal Move Profile (AKA Drive Method)
    // It will take in three static values: Distance, Max Speed, and Direction.
    // It will then use those values to travel the distance, accelerating up to the max speed for the first third of the distance,
    // maintaining that speed the next third, and then decelerating to a stop the last third to reduce impact shock and wheel slippage
    public void Drive(int Distance,float Direction) {

        double Inches = Distance * Direction;

        double WHEEL_DIAMETER = 5; //Diameter of the wheel in inches
        int ENCODER_CPR = 1440; // Encoder counts per Rev

        double CIRCUMFRENCE = Math.PI * WHEEL_DIAMETER;
        double ROTATIONS = Inches * CIRCUMFRENCE; // Number of rotations to drive
        double COUNTS = ENCODER_CPR * ROTATIONS; // Number of encoder counts to drive
        double TARGET = COUNTS / 1.8;

        telemetry.addData("Target", TARGET);
        telemetry.addData("Position", LeftDrive1.getCurrentPosition());

        LeftDrive1.setTargetPosition(LeftDrive1.getCurrentPosition()+(int) TARGET);
        LeftDrive2.setTargetPosition(LeftDrive2.getCurrentPosition()+(int) TARGET);
        RightDrive1.setTargetPosition(RightDrive1.getCurrentPosition()+(int) TARGET);
        RightDrive2.setTargetPosition(RightDrive2.getCurrentPosition() + (int) TARGET);

        LeftDrive1.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        LeftDrive2.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        RightDrive1.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        RightDrive2.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        if ((Math.abs (LeftDrive1.getCurrentPosition ()) > (Math.abs(TARGET)-5))&&
                (Math.abs (RightDrive1.getCurrentPosition ()) > (Math.abs(TARGET)-5)))
        {
            LeftDrive1.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            LeftDrive2.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            RightDrive1.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            RightDrive2.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        }


        if (Direction < 0 ) {
            LeftDrive1.setPower(.8);
            LeftDrive2.setPower(.8);
            RightDrive1.setPower(.8);
            RightDrive2.setPower(.8);
        }
        else {
            LeftDrive1.setPower(-.8);
            LeftDrive2.setPower(-.8);
            RightDrive1.setPower(-.8);
            RightDrive2.setPower(-.8);
        }

        if ((Math.abs (LeftDrive1.getCurrentPosition ()) > (Math.abs(TARGET)-5))&&
                (Math.abs (RightDrive1.getCurrentPosition ()) > (Math.abs(TARGET)-5)))
        {
            LeftDrive1.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            LeftDrive2.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            RightDrive1.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            RightDrive2.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
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

        sensorGyro.calibrate();
        while (sensorGyro.isCalibrating()) {
            try {
                Thread.sleep(50);
            } catch (InterruptedException ex){
                Thread.currentThread().interrupt();
            }
        }

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
            telemetry.addData("Text","Turning point reached");
        }
        else {
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
    public void ColorRecognition() {

        colorSensor.enableLed(false);
        float hsvValues[] = {0,0,0};
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
        if (colorSensor.red () >= 1) {
            // extend left button pusher
            telemetry.addData("Text","Red Detected");
        }
        else if (colorSensor.blue () >= 1) {
            // extend right button pusher
            telemetry.addData("Text","Blue Detected");
        }
        else {
            // extend lift two inches
            Lift.setPower(1);
            try {
                Thread.sleep(2000);
            } catch(InterruptedException ex) {
                Thread.currentThread().interrupt();
            }
            Lift.setPower(0);

            telemetry.addData("Text","None Detected");
            if (colorSensor.red () >= 1) {
                // extend left button pusher
                telemetry.addData("Text","Red Detected");
            }
            else if (colorSensor.blue () >= 1) {
                // extend right button pusher
                telemetry.addData("Text","Blue Detected");
            }
            else {
                // Maintain Servo Position
            }
        }
    }

    // This is the BreakBlock method. It sets the drivetrain to 0 power.
    public void BreakBlock(double Speed) {

        LeftDrive1.setPower(Speed);
        LeftDrive2.setPower(Speed);
        RightDrive1.setPower(Speed);
        RightDrive2.setPower(Speed);
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
        Delivery = hardwareMap.dcMotor.get("Delivery");

        ScoopLeft = hardwareMap.servo.get("ScoopLeft");
        ScoopRight = hardwareMap.servo.get("ScoopRight");
        ClimberLeft = hardwareMap.servo.get("ClimberLeft");
        ClimberRight = hardwareMap.servo.get("ClimberLeft");
        hardwareMap.logDevices();
        opticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("OpticalDistanceSensor");
        opticalDistanceSensor2 = hardwareMap.opticalDistanceSensor.get("OpticalDistanceSensor2");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        sensorGyro = hardwareMap.gyroSensor.get("gyro");
        cdim = hardwareMap.deviceInterfaceModule.get ("Device Interface Module 1");

        sensorGyro.calibrate();

        while (opModeIsActive()){
            telemetry.addData("Heading", sensorGyro.getHeading());
        }

        LeftDrive1 = hardwareMap.dcMotor.get("Motor1");
        LeftDrive2 = hardwareMap.dcMotor.get("Motor2");
        RightDrive1 = hardwareMap.dcMotor.get("Motor3");
        RightDrive2 = hardwareMap.dcMotor.get("Motor4");
        Sweeper = hardwareMap.dcMotor.get("Sweeper");
        Arm = hardwareMap.dcMotor.get("Arm");
        Lift = hardwareMap.dcMotor.get("Lift");
        ClimberLeft = hardwareMap.servo.get("LeftClimber");
        ClimberRight = hardwareMap.servo.get("RightClimber");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        RightDrive1.setDirection(DcMotor.Direction.REVERSE);
        RightDrive2.setDirection(DcMotor.Direction.REVERSE);

        LeftClimber = (.1);
        RightClimber = (0);

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
        RightDrive1.setPower(-.1);
        RightDrive2.setPower(-.1);
        sleep(2700);
        BreakBlock(0);



        LeftDrive1.setPower(1);
        LeftDrive2.setPower(1);
        RightDrive1.setPower(-1);
        RightDrive2.setPower(-1);
        sleep(1000);
        BreakBlock(0);

        LeftDrive1.setPower(-1);
        LeftDrive2.setPower(-1);
        RightDrive1.setPower(-1);
        RightDrive2.setPower(-1);
        sleep(1000);
        BreakBlock(0);

    }

}

