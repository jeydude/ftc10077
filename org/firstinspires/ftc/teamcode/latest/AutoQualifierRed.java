package org.firstinspires.ftc.teamcode.latest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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
@Autonomous(name = "QualifierRed", group = "Drive")
public class AutoQualifierRed extends LinearOpMode {

    // Drive motors
    private DcMotorEx fl, fr, bl, br, intake, belt, shooterLeft, shooterRight;
    // Analog servo kicker
    private Servo kicker, topKicker;
    // Timer for non-blocking delays
    private ElapsedTime stateTimer = new ElapsedTime();
    // -------------------- STATE MACHINE --------------------
    private enum AutoState {
        START,
        BALL1_KICK,
        BALL2_FEED,
        BALL2_KICK,
        BALL3_FEED,
        BALL3_KICK,

        TURN_TO_COLLECT,
        STRAFE_TO_COLLECT,

        COLLECT_BALLS,
        ADJUST_BALLS,
        DRIVE_BACK_TO_SHOOT,
        STRAFE_BACK_TO_SHOOT,
        TURN_BACK_TO_SHOOT,

        BALL4_KICK,
        BALL5_FEED,
        BALL5_KICK,
        BALL6_FEED,
        BALL6_KICK,

        STOP,
        DONE
    }
    private AutoState state = AutoState.START;
    
    // -------------------- CONSTANTS --------------------

    // Encoder ticks per motor revolution (GoBILDA 312 RPM)
    private static final double TICKS_PER_REV = 537.6;

    // Mecanum wheel diameter in inches
    private static final double WHEEL_DIAMETER_INCHES = 4.0;

    // Gear ratio (1.0 = direct drive)
    private static final double GEAR_RATIO = 1.0;
    // Correction factor to improve distance accuracy
    private static final double CORRECTION_VALUE = 0.92;
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

    // Default driving speed
    private static final double DRIVE_SPEED = 0.7;

    // Kicker servo positions (TUNE ON ROBOT)
    private static final double BOTTOM_KICKER_DOWN = 0.5;  // kicker down
    private static final double BOTTOM_KICKER_UP = 0;  // ball pushed
    private static final double TOP_KICKER_DOWN = 0.6; 
    private static final double TOP_KICKER_UP = 0.90;
    

    private double shooterPower = 1.0; // full power
    private double batteryVoltage = 12.0; // full power
    // Flag to indicate if a movement command has started
    private boolean moveStarted = false;
    private static int KICK_BALL_TIME = 900;
    private static int IS_RED=1;

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

        stateTimer.reset();
        while (opModeIsActive() && state != AutoState.DONE) {
            switch (state) {
                case START:
                    shooterOn(shooterPower);
                    if (!moveStarted) {
                        drive(49, DRIVE_SPEED);
                        moveStarted = true;
                    }
                    if (driveCompleted()) {
                        moveStarted = false;
                        state = AutoState.BALL1_KICK;
                        stateTimer.reset();
                    }
                    break;

                case BALL1_KICK:
                    runKicker();
                    if (stateTimer.milliseconds() > KICK_BALL_TIME+100) {
                        state = AutoState.BALL2_FEED;
                        stateTimer.reset();
                    }
                    break;

                case BALL2_FEED:
                    // topKicker.setPosition(TOP_KICKER_UP);
                    beltOn(0.8);
                    if (stateTimer.milliseconds() > 1000) {
                        beltOff();
                        state = AutoState.BALL2_KICK;
                        stateTimer.reset();
                    }
                    break;

                case BALL2_KICK:
                    runKicker();
                    if (stateTimer.milliseconds() > KICK_BALL_TIME+100) {
                        state = AutoState.BALL3_FEED;
                        stateTimer.reset();
                    }
                    break;

                case BALL3_FEED:
                    // topKicker.setPosition(TOP_KICKER_UP);
                    intakeOn(1.0); beltOn(0.8);
                    if (stateTimer.milliseconds() > 1000) {
                        beltOff(); intakeOff();
                        state = AutoState.BALL3_KICK;
                        stateTimer.reset();
                    }
                    break;

                case BALL3_KICK:
                    runKicker();
                    if (stateTimer.milliseconds() > KICK_BALL_TIME+100) {
                        shooterOff();
                        state = AutoState.TURN_TO_COLLECT;
                        stateTimer.reset();
                    }
                    break;
                case TURN_TO_COLLECT:
                    if (!moveStarted) {
                        turn(120*IS_RED, 0.5);//120 for RED
                        moveStarted = true;
                    }
                    if (driveCompleted()) {
                        moveStarted = false;
                        state = AutoState.STRAFE_TO_COLLECT;
                    }
                    break;
                case STRAFE_TO_COLLECT:
                    if (!moveStarted) {
                        strafe(8*IS_RED, 0.5);
                        moveStarted = true;
                    }
                    if (driveCompleted()) {
                        moveStarted = false;
                        state = AutoState.COLLECT_BALLS;
                    }
                    break;
                case COLLECT_BALLS:
                    topKicker.setPosition(TOP_KICKER_UP+0.05);
                    intakeOn(1);
                    beltOn(0.7);
                    if (!moveStarted) {
                        drive(38.0, 0.9);
                        moveStarted = true;
                    }
                    if (driveCompleted()) {
                        moveStarted = false;
                        intakeOff();
                        beltOff();
                        state = AutoState.ADJUST_BALLS;
                        stateTimer.reset();
                    }
                    break;

                case ADJUST_BALLS:
                    state = AutoState.DRIVE_BACK_TO_SHOOT;
                    topKicker.setPosition(TOP_KICKER_UP);
                    break;

                case DRIVE_BACK_TO_SHOOT:
                    if (!moveStarted) {
                        drive(-39, DRIVE_SPEED);
                        moveStarted = true;
                    }
                    if (driveCompleted()) {
                        moveStarted = false;
                        state = AutoState.STRAFE_BACK_TO_SHOOT;
                    }
                    break;
                case STRAFE_BACK_TO_SHOOT:
                    if (!moveStarted) {
                        strafe(-8*IS_RED, 0.8);
                        moveStarted = true;
                    }
                    if (driveCompleted()) {
                        moveStarted = false;
                        state = AutoState.TURN_BACK_TO_SHOOT;
                    }
                    break;
                case TURN_BACK_TO_SHOOT:
                    if (!moveStarted) {
                        shooterOn(shooterPower);
                        turn(-118*IS_RED, 0.5); //-120 for RED
                        moveStarted = true;
                    }
                    if (driveCompleted()) {
                        moveStarted = false;
                        state = AutoState.BALL4_KICK;
                        stateTimer.reset();
                    }
                    break;
                case BALL4_KICK:
                    runKicker();
                    if (stateTimer.milliseconds() > KICK_BALL_TIME+200) {
                        state = AutoState.BALL5_FEED;
                        stateTimer.reset();
                    }
                    break;

                case BALL5_FEED:
                    // topKicker.setPosition(TOP_KICKER_UP);
                    beltOn(0.8);
                    if (stateTimer.milliseconds() > 1000) {
                        beltOff();
                        state = AutoState.BALL5_KICK;
                        stateTimer.reset();
                    }
                    break;

                case BALL5_KICK:
                    runKicker();
                    if (stateTimer.milliseconds() > KICK_BALL_TIME+100) {
                        state = AutoState.BALL6_FEED;
                        stateTimer.reset();
                    }
                    break;

                case BALL6_FEED:
                    intakeOn(1); beltOn(0.8); 
                    if (stateTimer.milliseconds() > 1000) {
                        beltOff(); intakeOff(); 
                        state = AutoState.BALL6_KICK;
                        stateTimer.reset();
                    }
                    break;

                case BALL6_KICK:
                    runKicker();
                    if (stateTimer.milliseconds() > KICK_BALL_TIME+100) {
                        shooterOff();
                        state = AutoState.STOP;
                        moveStarted = false;
                        stateTimer.reset();
                        
                    }
                    break;
                case STOP:
                    if (!moveStarted) {
                        strafe(-20*IS_RED, 1.0);
                        moveStarted = true;
                    }
                    if (driveCompleted()) {
                        moveStarted = false;
                        stopDrive();
                        state = AutoState.DONE;
                    }
                    break;
            }
            telemetry.addData("State", state);
            telemetry.addData("Shooter Power", shooterPower);
            telemetry.addData("Battery Voltage", batteryVoltage);        
            telemetry.update();
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
        topKicker = hw.get(Servo.class, "topkicker");
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
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Intake should coast or brake depending on your design
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Shooter motors should coast for smoother RPM
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Brake so ball doesn't roll backward when stopped
        belt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  
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
        if (!moveStarted) { 

            // Convert inches to encoder ticks
            int ticks = (int) (inches * TICKS_PER_INCH);

            // All motors move the same amount
            setTargets(ticks, ticks, ticks, ticks);

            // Execute the movement
            runToPosition(power);
            // runToPositionWithBrake(power);
            moveStarted = true;
        }
    }

    /*
     * Strafes the robot left or right
     * @param inches  Distance to move (+ right, - left)
     * @param power   Motor power
     */
    private void strafe(double inches, double power) {
        if (!moveStarted) {
            // Apply strafe compensation
            int ticks = (int) (inches * TICKS_PER_INCH * STRAFE_MULTIPLIER);

            // Mecanum wheel strafing pattern
            setTargets(
                    ticks, -ticks,
                -ticks,  ticks
            );
            runToPosition(power);
            // runToPositionWithBrake(power);
            moveStarted = true;
        }
    }

    /*
     * Turns the robot in place
     * @param degrees  Degrees to turn (+ clockwise, - counterclockwise)
     * @param power    Motor power
     */
    private void turn(double degrees, double power) {
        if (!moveStarted) {
            // Convert degrees to encoder ticks
            int ticks = (int) (degrees * TICKS_PER_DEGREE);

            // Left and right sides move opposite directions
            setTargets(
                    ticks, -ticks,
                    ticks, -ticks
            );

            runToPosition(power);
            // runToPositionWithBrake(power);
            moveStarted = true;
        }
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
        DcMotorEx[] motors = new DcMotorEx[]{fl, fr, bl, br};
        for (DcMotorEx m : motors) {
            if (m.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            m.setPower(power);
        }
    }

    /*
     * Checks if all drive motors have reached their targets.
     * @return true if all motors are no longer busy
     */
    private boolean driveCompleted() {
        return !fl.isBusy() && !fr.isBusy() && !bl.isBusy() && !br.isBusy();
    }

    /*
     * Stops all drive motors.
     * Because braking is enabled, the robot will hold position.
     */
    private void stopDrive() {
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
     * Moves the servo up/down to kick one ball,
     * then returns it to the rest position.
     */
    private void runKicker() {
        double t = stateTimer.milliseconds();
        topKicker.setPosition(TOP_KICKER_DOWN);
        if (t >= 300) {
            kicker.setPosition(BOTTOM_KICKER_UP);
        }
        if (t >= KICK_BALL_TIME) {
            kicker.setPosition(BOTTOM_KICKER_DOWN);
            topKicker.setPosition(TOP_KICKER_UP);
            // beltOff();
        }
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
        else return 0.97;
    }

}
