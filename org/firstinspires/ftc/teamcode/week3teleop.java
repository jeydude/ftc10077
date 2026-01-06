package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="Qualifier Teleop", group="Linear Opmode")
public class week3teleop extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor frontLeft, frontRight, backLeft , backRight;
    private DcMotor intake, shooterLeft, shooterRight, belt;
    private Servo kicker, topKicker;

    private double shooterPower = 0.88;
    private double batteryVoltage = 12.5; // full power
    // ---------------- AUTO SHOOT ----------------
    ElapsedTime stateTimer = new ElapsedTime();
    private static final double BOTTOM_KICKER_DOWN = 0.75;
    private static final double BOTTOM_KICKER_UP = 0.40;
    private static final double TOP_KICKER_DOWN = 0.03;
    private static final double TOP_KICKER_UP = 0.28;
    private static int KICK_BALL_TIME = 1000;
    private double topKickerDownPosition = TOP_KICKER_DOWN;
    private double topKickerUpPosition = TOP_KICKER_UP;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;

    private enum AutoState {
        START,
        ADJUST_BALL1,
        BALL1_FEED,
        BALL1_KICK,
        PAUSE_AFTER_BALL1,
        BALL2_FEED,
        BALL2_KICK,
        BALL3_FEED,
        BALL3_KICK,
        BALL4_FEED,
        BALL4_KICK,
        DONE
    }

    private AutoState state = AutoState.DONE;
    
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables.
        initDrive(hardwareMap);
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match
        while (opModeIsActive()) {
            
            // Get joystick inputs
            double drive = gamepad1.left_stick_y;  // Forward/backward
            double strafe = -gamepad1.left_stick_x; // Left/right strafing
            double turn = gamepad1.right_stick_x;  // Turning

            // Calculate motor powers based on mecanum kinematics
            double frontLeftPower = drive + strafe + turn;
            double frontRightPower = drive - strafe - turn;
            double backLeftPower = drive - strafe + turn;
            double backRightPower = drive + strafe - turn;

            // Normalize the wheel speeds to keep them within the -1.0 to 1.0 range
            double maxPower = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            maxPower = Math.max(maxPower, Math.abs(backLeftPower));
            maxPower = Math.max(maxPower, Math.abs(backRightPower));

            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backLeftPower /= maxPower;
                backRightPower /= maxPower;
            }

            // Set motor powers
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);
            //Driver uses gamepad 1
            //Shooter uses gamepad 2
            if (gamepad1.right_bumper) { 
                intake.setPower(1.0); 
                topKicker.setPosition(topKickerUpPosition);
            } else if (gamepad1.right_trigger > 0.1) { 
                intake.setPower(-1.0);
            } else {
                intake.setPower(0.0);
            }
            //manage the shoot speed based on the dpad button pressed
            if (gamepad1.dpad_up) { shooterPower=1;}
            if (gamepad1.dpad_right) {shooterPower=0.9;}
            if (gamepad1.dpad_down) {shooterPower=0.85;}
            if (gamepad1.dpad_left) {shooterPower=0.8;}
            // ---------- AUTO FRONT SHOOT CONTROLS ----------

            //To shoot from the front shooting line
            if (gamepad1.x) {
                batteryVoltage = getBatteryVoltage();
                shooterPower = calculateFrontShooterPower(batteryVoltage);
                stateTimer.reset();
                state = AutoState.START;
                runAutoShoot();
                shooterPower = calculateShooterPower(batteryVoltage);
            }

            //To shoot from closure range
            if (gamepad1.y) {
                topKickerDownPosition = 0.05;
                batteryVoltage = getBatteryVoltage();
                shooterPower = calculateShooterPower(batteryVoltage);
                shooterPower = shooterPower-0.01;
                stateTimer.reset();
                state = AutoState.START;
                runAutoShoot();
                topKickerDownPosition = TOP_KICKER_DOWN;
                topKickerUpPosition = TOP_KICKER_UP;
                topKicker.setPosition(topKickerUpPosition);
            }
            
            //Shooter controls
            if (gamepad2.left_bumper) { 
                belt.setPower(1);
            } else if (gamepad2.left_trigger > 0.1) { 
                belt.setPower(-1);
            } else {
                belt.setPower(0);
            }

            //To reset the Top Kicker position if it was changed by DPAD
            if (gamepad2.b) {
                topKickerDownPosition = TOP_KICKER_DOWN;
                topKickerUpPosition = TOP_KICKER_UP;
                topKicker.setPosition(topKickerUpPosition);
                kicker.setPosition(BOTTOM_KICKER_DOWN);
                shooterPower = calculateShooterPower(batteryVoltage);
                shooterPower = shooterPower;
            }
            
            // DPAD UP – increase once per press
            if (gamepad2.dpad_up && !prevDpadUp) {
                double TOP_KICKER_POSITION = topKicker.getPosition();
                TOP_KICKER_POSITION = Math.min(TOP_KICKER_UP, TOP_KICKER_POSITION + 0.01);
                topKickerDownPosition = TOP_KICKER_POSITION;
                topKickerUpPosition = TOP_KICKER_POSITION;
                topKicker.setPosition(TOP_KICKER_POSITION);
            }
            
            // DPAD DOWN – decrease once per press
            if (gamepad2.dpad_down && !prevDpadDown) {
                double TOP_KICKER_POSITION = topKicker.getPosition();
                TOP_KICKER_POSITION = Math.max(TOP_KICKER_DOWN, TOP_KICKER_POSITION - 0.01);
                topKickerDownPosition = TOP_KICKER_POSITION;
                topKickerUpPosition = TOP_KICKER_POSITION;
                topKicker.setPosition(TOP_KICKER_POSITION);
            }
            
            // Save previous states
            prevDpadUp = gamepad2.dpad_up;
            prevDpadDown = gamepad2.dpad_down;

            
            // ---------- AUTO SHOOT CONTROLS ----------
            if (gamepad2.x) {
                state = AutoState.DONE;
            }

            //To shoot from the back middle shooting line
            if (gamepad2.y) {
                batteryVoltage = getBatteryVoltage();
                shooterPower = calculateShooterPower(batteryVoltage);
                stateTimer.reset();
                state = AutoState.START;
                runAutoShoot();
            }
            else if (gamepad2.right_bumper) {
                shooterOn(shooterPower);
                topKicker.setPosition(topKickerDownPosition);
                sleep(1000);
                kicker.setPosition(BOTTOM_KICKER_UP); // Example: Move to position 0
                sleep(KICK_BALL_TIME);
                kicker.setPosition(BOTTOM_KICKER_DOWN);
                topKicker.setPosition(topKickerUpPosition);
                shooterOff();
            }

            
            // Telemetry for debugging (optional)
            telemetry.addData("Status", "Running");
            telemetry.addData("Front Left Power", frontLeftPower);
            telemetry.addData("Front Right Power", frontRightPower);
            telemetry.addData("Back Left Power", backLeftPower);
            telemetry.addData("Back Right Power", backRightPower);
            telemetry.addData("Kicker Servo Position", kicker.getPosition());
            telemetry.addData("Ramp Servo Position", topKicker.getPosition());
            telemetry.addData("State", state);
            telemetry.addData("Shooter Power", shooterPower);
            telemetry.addData("Battery Voltage", batteryVoltage);
            telemetry.update();

        }
    }

    private void initDrive(HardwareMap hw) {
        // Initialize the hardware variables.
        frontLeft  = hardwareMap.get(DcMotor.class, "leftfront");
        frontRight = hardwareMap.get(DcMotor.class, "rightfront");
        backLeft  = hardwareMap.get(DcMotor.class, "leftrear");
        backRight = hardwareMap.get(DcMotor.class, "rightrear");
        intake = hardwareMap.get(DcMotor.class, "frontintake");
        shooterLeft = hardwareMap.get(DcMotor.class, "leftshooter");
        shooterRight = hardwareMap.get(DcMotor.class, "rightshooter");
        belt = hardwareMap.get(DcMotor.class, "belt");
        // Initialize the servo from the hardware map
        kicker = hardwareMap.get(Servo.class, "ballkicker"); 
        topKicker = hardwareMap.get(Servo.class, "topkicker"); 

        frontLeft.setDirection(DcMotor.Direction.REVERSE);//reverse
        backLeft.setDirection(DcMotor.Direction.FORWARD);//forward
        frontRight.setDirection(DcMotor.Direction.FORWARD);//forward
        backRight.setDirection(DcMotor.Direction.REVERSE);//reverse
        intake.setDirection(DcMotor.Direction.FORWARD);
        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterRight.setDirection(DcMotor.Direction.FORWARD);
        
        // Set all motors to run without encoders for simpler teleop control.
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Set all motors to brake mode
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);        
    }
    
    private void intakeOn(double power) {intake.setPower(power); }
    private void intakeOff() {intake.setPower(0);}
    private void beltOn(double power) { belt.setPower(power); }
    private void beltOff() { belt.setPower(0); }
    private void shooterOn(double power) {
        shooterLeft.setPower(power-0.1);
        shooterRight.setPower(power);
    }
    private void shooterOff() {
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
    }

    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        // Loop through all voltage sensors on the hardware map
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
    
    private double calculateShooterPower(double voltage) {
        double v = voltage;

        if (v >= 14.0) return 0.78;
        else if (v >= 13.9) return 0.79;
        else if (v >= 13.5) return 0.80;
        else if (v >= 13.3) return 0.80;
        else if (v >= 13.2) return 0.81;
        else if (v >= 13.1) return 0.82;
        else if (v >= 13.0) return 0.83;
        else if (v >= 12.9) return 0.84;
        else if (v >= 12.7) return 0.85;
        else if (v >= 12.5) return 0.86;
        else if (v >= 12.3) return 0.87;
        else if (v >= 12.0) return 0.89;
        else return 0.95;
    }
    private double calculateFrontShooterPower(double voltage) {
        double v = voltage;
        if (v >= 14.0) return 0.96;
        else if (v >= 13.5) return 0.97;
        else if (v >= 13.0) return 0.98;
        else return 0.99;
    }
    
    
    // ---------------- AUTO SHOOT LOGIC ----------------
    void runAutoShoot() {
        while (opModeIsActive() && state != AutoState.DONE) {
            switch (state) {
                case START:
                    topKicker.setPosition(topKickerDownPosition);
                    shooterOn(shooterPower);
                    if (stateTimer.milliseconds() >  KICK_BALL_TIME-500) {
                        state = AutoState.ADJUST_BALL1;
                        stateTimer.reset();
                    }
                    break;
                case ADJUST_BALL1:
                    topKicker.setPosition(topKickerUpPosition-0.03);
                    if (stateTimer.milliseconds() > KICK_BALL_TIME - 500) {
                        state = AutoState.BALL1_FEED;
                        stateTimer.reset();
                    }
                    break;
                case BALL1_FEED:
                    beltOn(0.5);
                    if (stateTimer.milliseconds() > KICK_BALL_TIME - 500) {
                        beltOff();
                        state = AutoState.BALL1_KICK;
                        stateTimer.reset();
                    }
                    break;
                case BALL1_KICK:
                    runKicker();
                    if (stateTimer.milliseconds() > KICK_BALL_TIME+200) {
                        state = AutoState.PAUSE_AFTER_BALL1;
                        stateTimer.reset();
                    }
                    break;
                case PAUSE_AFTER_BALL1:
                    if (stateTimer.milliseconds() > 300) {
                        state = AutoState.BALL2_FEED;
                        stateTimer.reset();
                    }
                    break;
                case BALL2_FEED:
                    beltOn(0.8);
                    if (stateTimer.milliseconds() > 1000) {
                        beltOff();
                        state = AutoState.BALL2_KICK;
                        stateTimer.reset();
                    }
                    break;

                case BALL2_KICK:
                    runKicker();
                    if (stateTimer.milliseconds() > KICK_BALL_TIME+200) {
                        state = AutoState.BALL3_FEED;
                        stateTimer.reset();
                    }
                    break;

                case BALL3_FEED:
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
                        state = AutoState.BALL4_FEED;
                        stateTimer.reset();
                    }
                    break;
                case BALL4_FEED:
                    intakeOn(1.0); beltOn(0.8);
                    if (stateTimer.milliseconds() > 1000) {
                        beltOff(); intakeOff();
                        state = AutoState.BALL4_KICK;
                        stateTimer.reset();
                    }
                    break;
                case BALL4_KICK:
                    runKicker();
                    if (stateTimer.milliseconds() > KICK_BALL_TIME+100) {
                        shooterOff();
                        state = AutoState.DONE;
                        stateTimer.reset();
                    }
                    break;
                case DONE:
                    stopAll();
                    break;
            }
        } //while
    }

    /*
     * Moves the servo up/down to kick one ball,
     * then returns it to the rest position.
     */
    private void runKicker() {
        double t = stateTimer.milliseconds();
        topKicker.setPosition(topKickerDownPosition);
        if (t >= 300) {
            // beltOn(0.7);
            kicker.setPosition(BOTTOM_KICKER_UP);
        }
        if (t >= KICK_BALL_TIME - 200) {
            kicker.setPosition(BOTTOM_KICKER_DOWN);
            // beltOff();
        }
        if (t >= KICK_BALL_TIME) {
            topKicker.setPosition(topKickerUpPosition);
            // beltOff();
        }
    }
    
    void stopAll() {
        shooterOff();
        beltOff();
        intakeOff();
        kicker.setPosition(BOTTOM_KICKER_DOWN);
        topKicker.setPosition(topKickerUpPosition);
    }
   
}
