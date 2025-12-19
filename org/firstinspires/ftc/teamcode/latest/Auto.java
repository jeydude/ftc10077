package org.firstinspires.ftc.teamcode.latest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * Autonomous OpMode using mecanum wheels with encoders.
 * Hardware:
 *  - GripForce mecanum wheels (4")
 *  - GoBILDA 312 RPM motors (537.6 ticks/rev)
 *
 * Features:
 *  - Drive forward/backward
 *  - Strafe left/right
 *  - Turn in place
 *  - Encoder-based movement
 *  - Motor braking when stopped
 */
@Autonomous(name = "AutoLatestRed", group = "Drive")
public class Auto extends LinearOpMode {

    // Drive motors
    private DcMotorEx fl, fr, bl, br, intake, belt, shooterLeft, shooterRight;
    // Analog servo kicker
    private Servo kicker;

    // -------------------- CONSTANTS --------------------

    // Encoder ticks per motor revolution (GoBILDA 312 RPM)
    private static final double TICKS_PER_REV = 537.6;

    // Mecanum wheel diameter in inches
    private static final double WHEEL_DIAMETER_INCHES = 4.0;

    // Gear ratio (1.0 = direct drive)
    private static final double GEAR_RATIO = 1.0;
    // Correction factor to improve distance accuracy
    private static final double CORRECTION_VALUE = 0.96;
    // Default driving speed
    private static final double DRIVE_SPEED = 0.7;
    // Encoder ticks required to move the robot 1 inch
    // Adjust based on your robot's performance
    // Calculate as (ticks per rev * gear ratio) / (wheel circumference)
    // wheel circumference = pi * diameter
    // Thus, TICKS_PER_INCH = (TICKS_PER_REV * GEAR_RATIO) / (π * WHEEL_DIAMETER_INCHES)
    // Then multiply by correction factor
    private static final double TICKS_PER_INCH =
            ((TICKS_PER_REV * GEAR_RATIO) /
            (Math.PI * WHEEL_DIAMETER_INCHES))*CORRECTION_VALUE;
    
    // Strafing is less efficient due to mecanum rollers
    // This multiplier compensates for sideways slip
    private static final double STRAFE_MULTIPLIER = 1.1;

    // Approximate encoder ticks needed to rotate 1 degree
    // This must be tuned for your specific robot
    private static final double TICKS_PER_DEGREE =10.8;

    // Kicker servo positions (TUNE ON ROBOT)
    private static final double KICKER_REST = 0.5;  // ball held
    private static final double KICKER_KICK = -1;  // ball pushed

    private double shooterPower = 1.0; // full power
    private double batteryVoltage = 3.5; // full power

    // -------------------- OPMODE --------------------

    @Override
    public void runOpMode() {

        // Initialize motors and encoders
        initDrive(hardwareMap);

        telemetry.addLine("Autonomous Ready");
        telemetry.update();

        // Wait for the play button to be pressed
        waitForStart();
        // Get shooter speed
        batteryVoltage = getBatteryVoltage();
        shooterPower = calculateShooterPower(batteryVoltage);
        telemetry.addData("Initial Shooter Power", shooterPower);
        telemetry.addData("Initial Battery Voltage", batteryVoltage);
        telemetry.update();
        
        if (opModeIsActive()) {
            // ----- FIRST 3 BALLS -----
            //Spin up shooter ONCE
            shooterOn(shooterPower);
            // Drive backward 44 inches
            drive(44, DRIVE_SPEED);
            sleep(100);
            // -------- BALL 1 --------
            kickBall(); sleep(50); //kick ball taking ~1.4 seconds
            //first ball time is about 1.5 seconds

            // second ball preparation time is about 1 seconds
            beltOn(1); sleep(1000);
            beltOff(); sleep(50);

            // -------- BALL 2 --------
            kickBall(); sleep(50);
            // end of second ball total time is about 2.4 seconds


            //third ball preparation time is about 1 seconds
            intakeOn(1); beltOn(1); sleep(1000);
            beltOff(); intakeOff(); sleep(50);
            
            // -------- BALL 3 --------
            kickBall(); sleep(50);
            //third ball total time is about 2.4 seconds
            // Shut down shooter
            shooterOff();

            // ----- COLLECT NEXT 3 BALLS -----
            // Move to collect another 3 balls
            // Strafe right 18 inches
            
            // Drive backward 5 inches
            drive(5, 0.4); sleep(100);
            
            // Turn 120 degrees clockwise
            turn(-120, 0.45);
            
            strafe(-4.5, 0.5); //it was -5 inches
            sleep(100);

            // Start intake before moving
            intakeOn(1.0);  beltOn(0.5);
            // Drive forward to collect the ball
            drive(40, 0.3);
            // Give intake time to fully pull in the ball
            sleep(250);
            // Stop the intake after collecting the ball
            intakeOff(); beltOff();

            // Optional: clear last ball
            shooterOn(-0.5); intakeOn(-0.5); beltOn(-0.8); sleep(250);
            intakeOff(); beltOff(); shooterOff();

            // ----- RETURN TO SHOOTING POSITION -----
            batteryVoltage = getBatteryVoltage();
            shooterPower = calculateShooterPower(batteryVoltage);
            telemetry.addData("Shooter Power", shooterPower);
            telemetry.addData("Battery Level", batteryVoltage);
            telemetry.update();
            drive(-40, DRIVE_SPEED);  // Adjust distance based on field
            strafe(4.5, 0.5); // Strafe back to original lane
            // Spin up shooter
            shooterOn(shooterPower);
            turn(120, 0.45);  // Turn back to face goal

            // ----- SHOOT NEXT 3 BALLS -----
            // -------- BALL 4 --------
            kickBall(); sleep(150);
            //4th ball time is about 1.5 seconds

            // fifth ball preparation time is about 1 seconds
            beltOn(1); sleep(1000);
            beltOff(); sleep(50);
            // -------- BALL 5 --------
            kickBall(); sleep(50);
            // end of fifth ball total time is about 2.4 seconds

            //sixth ball preparation time is about 1 seconds
            intakeOn(1); beltOn(1); sleep(1000);
            beltOff(); intakeOff(); sleep(50);
            
            // -------- BALL 6 --------
            kickBall(); sleep(50);

            //sixth ball total time is about 2.4 seconds
            // Shut down shooter
            shooterOff();
            strafe(20, 1);

        }
    }

    // -------------------- INITIALIZATION --------------------

    /*
     * Maps motors, sets directions, enables braking,
     * and resets encoders.
     */
    private void initDrive(HardwareMap hw) {

        // Match these names to the Robot Configuration
        fl = hw.get(DcMotorEx.class, "leftfront");
        fr = hw.get(DcMotorEx.class, "rightfront");
        bl = hw.get(DcMotorEx.class, "leftrear");
        br = hw.get(DcMotorEx.class, "rightrear");
        // Intake motor (no encoder needed)
        intake = hw.get(DcMotorEx.class, "frontintake");
        belt = hw.get(DcMotorEx.class, "belt");
        kicker = hw.get(Servo.class, "ballkicker");
        shooterLeft = hw.get(DcMotorEx.class, "leftshooter");
        shooterRight = hw.get(DcMotorEx.class, "rightshooter");


        // Reverse right side motors so all wheels move forward together
        fr.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        // Usually one shooter motor must be reversed
        shooterLeft.setDirection(DcMotor.Direction.REVERSE);

        // Brake motors when power is set to zero
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Intake should coast or brake depending on your design
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Shooter motors should coast for smoother RPM
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Brake so ball doesn't roll backward when stopped
        belt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoder values to zero
        resetEncoders();
  
    }

    /*
     * Stops motors, resets encoder counts,
     * then switches back to encoder mode.
     */
    private void resetEncoders() {
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // -------------------- MOVEMENT METHODS --------------------

    /*
     * Drives the robot forward or backward
     * @param inches  Distance to move (+ forward, - backward)
     * @param power   Motor power (0.0 - 1.0)
     */
    private void drive(double inches, double power) {

        // Convert inches to encoder ticks
        int ticks = (int) (inches * TICKS_PER_INCH);

        // All motors move the same amount
        setTargets(ticks, ticks, ticks, ticks);

        // Execute the movement
        runToPosition(power);
        // runToPositionWithBrake(power);
    }

    /*
     * Strafes the robot left or right
     * @param inches  Distance to move (+ right, - left)
     * @param power   Motor power
     */
    private void strafe(double inches, double power) {

        // Apply strafe compensation
        int ticks = (int) (inches * TICKS_PER_INCH * STRAFE_MULTIPLIER);

        // Mecanum wheel strafing pattern
        setTargets(
                ticks, -ticks,
               -ticks,  ticks
        );

        runToPosition(power);
        // runToPositionWithBrake(power);
    }

    /*
     * Turns the robot in place
     * @param degrees  Degrees to turn (+ clockwise, - counterclockwise)
     * @param power    Motor power
     */
    private void turn(double degrees, double power) {

        // Convert degrees to encoder ticks
        int ticks = (int) (degrees * TICKS_PER_DEGREE);

        // Left and right sides move opposite directions
        setTargets(
                ticks, -ticks,
                ticks, -ticks
        );

        runToPosition(power);
        // runToPositionWithBrake(power);
    }

    // -------------------- LOW-LEVEL HELPERS --------------------

    /*
     * Adds target encoder positions to each motor.
     * Uses current position so movements can be chained.
     */
    private void setTargets(int flT, int frT, int blT, int brT) {

        fl.setTargetPosition(fl.getCurrentPosition() + flT);
        fr.setTargetPosition(fr.getCurrentPosition() + frT);
        bl.setTargetPosition(bl.getCurrentPosition() + blT);
        br.setTargetPosition(br.getCurrentPosition() + brT);
    }

    /*
     * Runs motors to their target positions
     * and blocks until movement is complete.
     */
    private void runToPosition(double power) {

        // Put motors into RUN_TO_POSITION mode
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(power);
        }

        // Wait until all motors finish moving
        while (opModeIsActive() &&
              (fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy())) {
            telemetry.addData("Shooter Power", shooterPower);
            telemetry.addData("Battery Voltage", batteryVoltage);
            telemetry.addData("FL Encoder", fl.getCurrentPosition());
            telemetry.addData("FR Encoder", fr.getCurrentPosition());
            telemetry.addData("BL Encoder", bl.getCurrentPosition());
            telemetry.addData("BR Encoder", br.getCurrentPosition());
            telemetry.update();
        }

        // Stop motors and hold position
        stopMotors();

        // Reset encoders so next move starts from zero
        resetEncoders();
    }

    /*
     * Stops all drive motors.
     * Because braking is enabled, the robot will hold position.
     */
    private void stopMotors() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    /*
    * Runs the intake to collect a ball
    */
    private void intakeOn(double power) {
        intake.setPower(power);
    }

    /*
    * Reverses the intake to spit out a ball (optional)
    */
    private void intakeReverse(double power) {
        intake.setPower(-power);
    }

    /*
    * Stops the intake motor
    */
    private void intakeOff() {
        intake.setPower(0);
    }

    /*
    * Runs belt forward to feed ball into shooter
    */
    private void beltOn(double power) {
        belt.setPower(power);
    }

    /*
    * Reverse belt to clear jams (optional)
    */
    private void beltReverse(double power) {
        belt.setPower(-power);
    }

    /*
    * Stops the belt motor
    */
    private void beltOff() {
        belt.setPower(0);
    }

    /*
    * Spins up both shooter motors
    */
    private void shooterOn(double power) {
        shooterLeft.setPower(power);
        shooterRight.setPower(power);
    }

    /*
    * Stops shooter motors
    */
    private void shooterOff() {
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
    }

    /*
    * Moves the servo forward to kick one ball,
    * then returns it to the rest position.
    */
    private void kickBall() {
        kicker.setPosition(KICKER_KICK);
        sleep(600);                 // time to fully kick ball
        beltOn(-0.8);               // reverse belt to prevent jams 
        sleep(300);                 // allow servo to return
        beltOff();                  // stop belt
        kicker.setPosition(KICKER_REST);
        sleep(400);                 // allow servo to return
    }

    // -------------------- POWER LOGIC --------------------
    private double getBatteryVoltage() {
        double voltage = 0;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            voltage = sensor.getVoltage();
            if (voltage > 0) break; // take the first valid reading
        }
        return voltage;
    }

    // private double calculateShooterPower() {
    //     double voltage = getBatteryVoltage();   // measured battery
    //     double minVoltage = 12.0;               // lowest voltage to hit full power
    //     double maxVoltage = 13.8;               // highest voltage to reduce power
    //     double minPower = 0.85;                 // shooter power at max voltage
    //     double maxPower = 1.0;                  // shooter power at min voltage

    //     // Linear mapping: higher voltage → lower power
    //     double power = maxPower - ((voltage - minVoltage) / (maxVoltage - minVoltage)) * (maxPower - minPower);

    //     // Clamp power between minPower and maxPower
    //     if (power > maxPower) power = maxPower;
    //     if (power < minPower) power = minPower;

    //     return power;
    // }

    private double calculateShooterPower(double voltage) {
        double v = voltage;

        if (v >= 14.0) return 0.83;
        else if (v >= 13.9) return 0.85;
        else if (v >= 13.5) return 0.865;
        else if (v >= 13.3) return 0.87;
        else if (v >= 13.2) return 0.88;
        else if (v >= 13.1) return 0.89;
        else if (v >= 13.0) return 0.90;
        else if (v >= 12.9) return 0.91;
        else if (v >= 12.7) return 0.92;
        else if (v >= 12.5) return 0.94;
        else if (v >= 12.0) return 0.95;
        else return 1.0;
    }

    private void runToPositionWithBrake(double power) {

        // Set RUN_TO_POSITION mode
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(power);
        }

        // Wait until all motors reach target
        while (opModeIsActive() && 
            (fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy())) {

            // Optional: telemetry for debugging
            telemetry.addData("Shooter Power", shooterPower);
            telemetry.addData("Battery Voltage", batteryVoltage);            
            telemetry.addData("FL", fl.getCurrentPosition());
            telemetry.addData("FR", fr.getCurrentPosition());
            telemetry.addData("BL", bl.getCurrentPosition());
            telemetry.addData("BR", br.getCurrentPosition());
            telemetry.update();
        }

        // Braking phase: reduce power gradually to zero
        double currentPower = power;
        int steps = 5;
        for (int i = steps; i >= 0; i--) {
            double p = currentPower * i / steps;
            fl.setPower(p);
            fr.setPower(p);
            bl.setPower(p);
            br.setPower(p);
            sleep(20); // short step
        }

        // Ensure motors are fully stopped
        stopMotors();

        // Reset encoders for next move
        resetEncoders();
    }
}
