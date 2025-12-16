package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
@Autonomous(name = "Autonomous", group = "Drive")
public class Autonomous extends LinearOpMode {

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

    // Encoder ticks required to move the robot 1 inch
    private static final double TICKS_PER_INCH =
            (TICKS_PER_REV * GEAR_RATIO) /
            (Math.PI * WHEEL_DIAMETER_INCHES);

    // Strafing is less efficient due to mecanum rollers
    // This multiplier compensates for sideways slip
    private static final double STRAFE_MULTIPLIER = 1.1;

    // Approximate encoder ticks needed to rotate 1 degree
    // This must be tuned for your specific robot
    private static final double TICKS_PER_DEGREE = 10.8;


    // Kicker servo positions (TUNE ON ROBOT)
    private static final double KICKER_REST = 0.5;  // ball held
    private static final double KICKER_KICK = -0.8;  // ball pushed

    // -------------------- OPMODE --------------------

    @Override
    public void runOpMode() {

        // Initialize motors and encoders
        initDrive(hardwareMap);

        telemetry.addLine("Autonomous Ready");
        telemetry.update();

        // Wait for the play button to be pressed
        waitForStart();

        if (opModeIsActive()) {

            // ----- FIRST 3 BALLS -----
            // Drive backward 24 inches
            drive(-24, 0.6);

            // Spin up shooter ONCE
            double shooterPower = calculateShooterPower();
            shooterOn(shooterPower);
            sleep(1500);

            // -------- BALL 1 --------
            kickBall();
            sleep(120);

            beltOn(0.5);
            sleep(250);
            beltOff();
            sleep(150);

            // -------- BALL 2 --------
            kickBall();
            sleep(120);

            intakeOn(0.8);
            beltOn(0.5);
            sleep(250);
            beltOff();
            intakeOff();
            sleep(150);
            

            // -------- BALL 3 --------
            kickBall();
            sleep(200);

            // Shut down shooter
            shooterOff();

            // ----- COLLECT NEXT 3 BALLS -----
            // Move to collect another 3 balls
            // Strafe right 18 inches
            strafe(18, 0.5);

            // Turn 90 degrees clockwise
            turn(90, 0.45);

            // Drive backward 12 inches
            drive(-12, 0.4);

            // Start intake before moving
            intakeOn(1.0);

            // Drive forward to collect the ball
            drive(30, 0.4);

            // Give intake time to fully pull in the ball
            sleep(300);

            // Stop the intake after collecting the ball
            intakeOff();

            // Optional: clear last ball
            intakeReverse(0.4);
            sleep(100);
            intakeOff();

            // ----- RETURN TO SHOOTING POSITION -----
            drive(-30, 0.4);  // Adjust distance based on field
            turn(-90, 0.45);  // Turn back to face goal
            strafe(-18, 0.5); // Strafe back to original lane
            drive(-12, 0.4);  // Final approach to shooting position
            // ----- SHOOT NEXT 3 BALLS -----
            // Spin up shooter
            shooterPower = calculateShooterPower();
            shooterOn(shooterPower);
            sleep(1500);
            // -------- BALL 4 --------
            kickBall();
            sleep(120);

            beltOn(0.5);
            sleep(250);
            beltOff();
            sleep(150);
            // -------- BALL 5 --------
            kickBall();
            sleep(120);

            intakeOn(0.8);
            beltOn(0.5);
            sleep(250);
            beltOff();
            intakeOff();
            sleep(150);
            // -------- BALL 6 --------
            kickBall();
            sleep(200);

            // Shut down shooter
            shooterOff();


        }
    }

    // -------------------- INITIALIZATION --------------------

    /*
     * Maps motors, sets directions, enables braking,
     * and resets encoders.
     */
    private void initDrive(HardwareMap hw) {

        // Match these names to the Robot Configuration
        fl = hw.get(DcMotorEx.class, "frontLeft");
        fr = hw.get(DcMotorEx.class, "frontRight");
        bl = hw.get(DcMotorEx.class, "backLeft");
        br = hw.get(DcMotorEx.class, "backRight");
        // Intake motor (no encoder needed)
        intake = hw.get(DcMotorEx.class, "intake");
        belt = hw.get(DcMotorEx.class, "belt");
        kicker = hw.get(Servo.class, "kicker");
        shooterLeft = hw.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hw.get(DcMotorEx.class, "shooterRight");


        // Reverse right side motors so all wheels move forward together
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
        // Usually one shooter motor must be reversed
        shooterRight.setDirection(DcMotor.Direction.REVERSE);

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
        
        // Intake does not need encoders
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Belt usually does not need encoders
        belt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Shooter does not need RUN_TO_POSITION
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
    * Moves the servo forward to kick one ball,
    * then returns it to the rest position.
    */
    private void kickBall() {
        kicker.setPosition(KICKER_KICK);
        sleep(180);                 // time to fully kick ball
        kicker.setPosition(KICKER_REST);
        sleep(150);                 // allow servo to return
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


    private double getBatteryVoltage() {
        double voltage = 0;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            voltage = sensor.getVoltage();
            if (voltage > 0) break; // take the first valid reading
        }
        return voltage;
    }

    private double calculateShooterPower() {
        double voltage = getBatteryVoltage();   // measured battery
        double minVoltage = 12.0;               // lowest voltage to hit full power
        double maxVoltage = 13.8;               // highest voltage to reduce power
        double minPower = 0.85;                 // shooter power at max voltage
        double maxPower = 1.0;                  // shooter power at min voltage

        // Linear mapping: higher voltage → lower power
        double power = maxPower - ((voltage - minVoltage) / (maxVoltage - minVoltage)) * (maxPower - minPower);

        // Clamp power between minPower and maxPower
        if (power > maxPower) power = maxPower;
        if (power < minPower) power = minPower;

        return power;
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