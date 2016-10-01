package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class RedCloseRampX10 extends LinearOpMode {

    DcMotor LeftDrive1;
    DcMotor LeftDrive2;
    DcMotor RightDrive1;
    DcMotor RightDrive2;
    DcMotor Lift;
    DcMotor Sweeper;
    DcMotor Arm;
    DcMotor Delivery;

    Servo FangLeft;
    Servo FangRight;
    Servo ClimberRight;
    Servo ClimberLeft;
    Servo DeliverClimbers;

    double LeftFang;
    double RightFang;
    double RightClimber;
    double LeftClimber;
    double ClimbersDeliver;

    ColorSensor colorSensor;
    GyroSensor sensorGyro;


    DeviceInterfaceModule cdim;



    // All the methods:

    /* This is the Drive Method
     It will take in two static values: Distance and Direction.
     It will then convert inches to encoder readings and drive the distance. */
    public void Drive(double Distance, double Direction) {
        double GEAR_RATIO = 2;
        int ENCODER_CPR = 1120; // Encoder counts per Rev
        double WHEEL_DIAMETER = 5.5; //Diameter of the wheel in inches
        double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        double ROTATIONS = Distance / CIRCUMFERENCE; // Number of rotations to 0drive
        double COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO * Direction; // Number of encoder counts to drive

        LeftDrive1.setTargetPosition(LeftDrive1.getCurrentPosition() + (int) COUNTS);
        LeftDrive2.setTargetPosition(LeftDrive2.getCurrentPosition() + (int) COUNTS);
        RightDrive1.setTargetPosition(RightDrive1.getCurrentPosition() + (int) COUNTS);
        RightDrive2.setTargetPosition(RightDrive2.getCurrentPosition() + (int) COUNTS);

        LeftDrive1.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        LeftDrive2.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        RightDrive1.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        RightDrive2.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        if(Direction == 1){
            while (COUNTS >= LeftDrive1.getCurrentPosition()){
                telemetry.addData("Counts", COUNTS);
                telemetry.addData("DriveLeft", LeftDrive1.getCurrentPosition());
                telemetry.addData("DriveRight", RightDrive1.getCurrentPosition());
                telemetry.addData("Direction", 1);
                LeftDrive1.setPower(1);
                LeftDrive2.setPower(1);
                RightDrive1.setPower(.85);
                RightDrive2.setPower(.85);
            }
        }
        else if(Direction == -1){
            while (COUNTS <= LeftDrive1.getCurrentPosition()-5){
                telemetry.addData("Counts", COUNTS);
                telemetry.addData("DriveLeft", LeftDrive1.getCurrentPosition());
                telemetry.addData("DriveRight", RightDrive1.getCurrentPosition());
                telemetry.addData("Direction", -1);
                LeftDrive1.setPower(1);
                LeftDrive2.setPower(1);
                RightDrive1.setPower(.85);
                RightDrive2.setPower(.85);
            }
        }
        LeftDrive1.setPower(0);
        LeftDrive2.setPower(0);
        RightDrive1.setPower(0);
        RightDrive2.setPower(0);

    }


    /*This is the Gyro Turn Method.
    It will take in a value of degrees to turn and a direction value (positive for right, negative for left)
    It will then us the Gyro Sensor to turn that many degrees in the correct direction*/
    // Function called in the init
    // Calibrates and does other preparations for the gyro sensor before autonomous
    // Needs nothing passed to it
    private void setUpGyro() throws InterruptedException {

        // setup the Gyro
        // write some device information (connection info, name and type)
        // to the log file.
        hardwareMap.logDevices();
        // get a reference to our GyroSensor object.
        sensorGyro = hardwareMap.gyroSensor.get("gyro");
        // calibrate the gyro.
        sensorGyro.calibrate();
        while (sensorGyro.isCalibrating())  {
            sleep(50);
        }
        // End of setting up Gyro
    }

    // function to use the gyro to do a spinning turn in place.
    // it points the robot at an absolute heading, not a relative turn.  0 will point robot to same
    // direction we were at the start of program.
    // Pass:
    // target_heading = the new heading we want to point robot at,
    // max_speed = the max speed the motor can run in the range of 0 to 1
    // direction = the direction we will turn, 1 is clockwaise, -1 is counter-clockwise
    // Returns:
    // heading = the new heading the gyro reports
    int gyro_turn2heading (int target_heading, double max_speed, int direction) throws InterruptedException {
        int error;
        double old_speed = 0;
        double speed = 0;
        double min_speed = 0.05;
        double acceleration = 0.01;
        double kp = 0.01;             // Proportional error constant

        int heading = sensorGyro.getHeading();

        LeftDrive1.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        LeftDrive2.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        RightDrive1.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        RightDrive2.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        while ( (!angle_in_range(heading, target_heading, 5, direction )  && opModeIsActive())) {
            // now calculate the speed we should be moving at
            old_speed = speed;                    // save our old speed for use later.

            heading = sensorGyro.getHeading();
            error = delta_angle(target_heading, heading, direction);

            speed = error * kp * direction;

            // limit the acceleration of the motors speed at beginning of turns.
            if( Math.abs(speed) > Math.abs(old_speed))
                speed = old_speed + (direction * acceleration);

            // set a minimum power for the motors to make sure they move
            if (Math.abs(speed) < min_speed)
                speed = min_speed * direction;


            // don't exceed the maximium speed requested
            if (Math.abs(speed) > max_speed)
                speed = max_speed * direction;

            // now set the motor speeds
            LeftDrive1.setPower(speed);
            LeftDrive2.setPower(speed);// if direction = 1 we want right side to back up.
            RightDrive1.setPower(-speed);
            RightDrive2.setPower(-speed);

            waitForNextHardwareCycle();
        }

        // done with the turn so shut off the motors now!
        LeftDrive1.setPower(0);
        LeftDrive2.setPower(0);
        RightDrive1.setPower(0);
        RightDrive2.setPower(0);
        sleep(100);
        telemetry.addData("1. h", heading);
        telemetry.addData("2. g", sensorGyro.getHeading());

        return (sensorGyro.getHeading());

    }
    // function to compute the difference between two angles
    // Pass:
    // a = first angle, must be in range of 0 to 359
    // b = second angle, must be in range of 0 to 359
    // direction = the direction we will turn, 1 is clockwise, -1 is counter-clockwise
    // returns:
    // the difference in the range of 0 to 359
    public static int delta_angle(int a, int b, int direction)  {
        int delta;

        if (direction == 1)
            delta = a - b;
        else
            delta = b - a;

        while (delta < 0)
            delta = delta + 360;
        while (delta > 360)
            delta = delta + 360;
        return (delta);
    }

    // function to determine if an angle is in range of another angle.
    // handles the troublesome issue of degrees going from 359 to 0
    // Pass:
    //		angle = angle to be tested
    //        start = start of range, must be in range of 0 to 359
    //        range = size of range to check in degrees
    //        direction = the direction we will turn, 1 is clockwise, -1 is counter-clockwise
    // returns:
    //        true if in range false if not in range
    public static boolean angle_in_range(int angle, int start, int range, int direction)  {

        int stop;
        boolean result;

        if (direction >= 1) {
            stop = start + range;
            if (stop <= 359 )
                result = (angle >= start && angle <= stop);
            else // we wrapped around from 359 to 0 degrees
                result = (angle >= start && angle <= 359) || (angle >=0 && angle <= (stop-360));
        } else {
            stop = start - range;
            if (stop >= 0 )
                result = (angle <= start && angle >= stop);
            else // we wrapped around from 0 to 359 degrees
                result = ((angle >= (stop+360)  && angle <=359) || (angle >= 0 && angle <= start));
        }

        return (result);
    }


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
    // This is the BrakeBlock method. It sets the drivetrain to 0 power.
    public void BrakeBlock(double Speed) {

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

        ClimberLeft = hardwareMap.servo.get("ClimberLeft");
        ClimberRight = hardwareMap.servo.get("ClimberRight");
        FangLeft = hardwareMap.servo.get("FangLeft");
        FangRight = hardwareMap.servo.get("FangRight");
        DeliverClimbers = hardwareMap.servo.get("DC");
        hardwareMap.logDevices();
        colorSensor = hardwareMap.colorSensor.get("Color");
        sensorGyro = hardwareMap.gyroSensor.get("gyro");
        cdim = hardwareMap.deviceInterfaceModule.get ("Device Interface Module 1");

        sensorGyro.calibrate();
        FangLeft.setPosition(.1);
        FangRight.setPosition(.9);

        LeftDrive1.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        LeftDrive2.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        RightDrive1.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        RightDrive2.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        waitForStart();

        sleep(10000);
        // Clear ramp
        Drive(47, 1);
        BrakeBlock(0);
        // Align perpendicular to ramp
        gyro_turn2heading(77, .6, 1);
        // Drive up ramp
        LeftDrive1.setPower(-.6);
        LeftDrive2.setPower(-.6);
        RightDrive1.setPower(-.5);
        RightDrive2.setPower(-.5);
        sleep(2000);
        BrakeBlock(0);
        // Latch on to churro
        FangLeft.setPosition(1);
        FangRight.setPosition(0);
        // Align to churro
        LeftDrive1.setPower(-.6);
        LeftDrive2.setPower(-.6);
        RightDrive1.setPower(-.5);
        RightDrive2.setPower(-.5);
        sleep(100);
        BrakeBlock(0);

    }
}
