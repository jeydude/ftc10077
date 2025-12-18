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

@TeleOp(name="Week 3 Teleop", group="Linear Opmode")
public class week3teleop  extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor intake = null;
    private DcMotor leftshooter = null;
    private DcMotor rightshooter = null;
    private DcMotor belt = null;
    private Servo kicker = null;
    private double shooter_speed = 1.0;
    // ---------------- AUTO SHOOT ----------------
    ElapsedTime shooterTimer = new ElapsedTime();

    enum AutoShootState {
        IDLE,
        SPIN_UP,
        SHOOT_1,
        FEED_2,
        SHOOT_2,
        FEED_3,
        SHOOT_3,
        DONE
    }

    AutoShootState shootState = AutoShootState.IDLE;
    boolean autoShooting = false;

    
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables.
        frontLeft  = hardwareMap.get(DcMotor.class, "leftfront");
        frontRight = hardwareMap.get(DcMotor.class, "rightfront");
        backLeft  = hardwareMap.get(DcMotor.class, "leftrear");
        backRight = hardwareMap.get(DcMotor.class, "rightrear");
        intake = hardwareMap.get(DcMotor.class, "frontintake");
        leftshooter = hardwareMap.get(DcMotor.class, "leftshooter");
        rightshooter = hardwareMap.get(DcMotor.class, "rightshooter");
        belt = hardwareMap.get(DcMotor.class, "belt");
        // Initialize the servo from the hardware map
        kicker = hardwareMap.get(Servo.class, "ballkicker"); 


        // Most robots need the motors on one side to be reversed to drive forward.
        // Reverse the appropriate motors to ensure all wheels rotate in the correct direction for forward movement.
        // frontLeft.setDirection(DcMotor.Direction.FORWARD);
        // backLeft.setDirection(DcMotor.Direction.FORWARD);
        // frontRight.setDirection(DcMotor.Direction.REVERSE);
        // backRight.setDirection(DcMotor.Direction.REVERSE);
        
        frontLeft.setDirection(DcMotor.Direction.REVERSE);//reverse
        backLeft.setDirection(DcMotor.Direction.FORWARD);//forward
        frontRight.setDirection(DcMotor.Direction.FORWARD);//forward
        backRight.setDirection(DcMotor.Direction.REVERSE);//reverse
        intake.setDirection(DcMotor.Direction.FORWARD);
        leftshooter.setDirection(DcMotor.Direction.REVERSE);
        rightshooter.setDirection(DcMotor.Direction.FORWARD);
        
        // Set all motors to run without encoders for simpler teleop control.
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftshooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightshooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Set all motors to brake mode
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftshooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightshooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
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
            if (gamepad1.right_bumper) { 
                intake.setPower(1.0); // Full power
            } else if (gamepad1.right_trigger > 0.1) { 
                intake.setPower(-1.0); // Full power backwards
            } else {
                intake.setPower(0.0); // Stop
            }
            
            if (gamepad2.left_bumper) { 
                belt.setPower(1);
            } else if (gamepad2.left_trigger > 0.1) { 
                belt.setPower(-1);
            } else {
                belt.setPower(0);
            }
            if (gamepad2.a) { 
                //When a is pressed, the kicker goes up
                kicker.setPosition(-0.40); // Example: Move to position 0
                telemetry.addData("Servo Position", "Up Position");
                sleep(800);
                kicker.setPosition(0.5);
                
            } else if (gamepad2.b) {
                kicker.setPosition(0.5); // Example: Move to position 0.5 (mid-range)
                telemetry.addData("Servo Position", "0.5 Position");
            }
            if (gamepad2.right_trigger>0.1) {
                leftshooter.setPower(-0.2);
                rightshooter.setPower(-0.2);
            } 
                        // ---------- AUTO SHOOT CONTROLS ----------
            if (gamepad2.x) {
                stopAllShooter();
                autoShooting = false;
                shootState = AutoShootState.IDLE;
            }

            if (gamepad2.y && !autoShooting) {
                autoShooting = true;
                shooterTimer.reset();
                shootState = AutoShootState.SPIN_UP;
            }

            if (autoShooting) {
                runAutoShoot();
            }
            else if (gamepad2.right_bumper) {
                leftshooter.setPower(shooter_speed);
                rightshooter.setPower(shooter_speed);
                sleep(1000);
                kicker.setPosition(-0.40); // Example: Move to position 0
                telemetry.addData("Servo Position", "Up Position");
                sleep(800);
                kicker.setPosition(0.5);
            }
            else {
                leftshooter.setPower(0);
                rightshooter.setPower(0);
            }
            //manage the shoot speed based on the dpad button pressed
            if (gamepad1.dpad_up) {
                    shooter_speed=1;
            }
            if (gamepad1.dpad_right) {
                shooter_speed=0.925;
            }
            if (gamepad1.dpad_down) {
                shooter_speed=0.9;
            }
            if (gamepad1.dpad_left) {
                shooter_speed=0.85;
            }
            
            
            // Telemetry for debugging (optional)
            telemetry.addData("Status", "Running");
            telemetry.addData("Front Left Power", frontLeftPower);
            telemetry.addData("Front Right Power", frontRightPower);
            telemetry.addData("Back Left Power", backLeftPower);
            telemetry.addData("Back Right Power", backRightPower);
            telemetry.addData("Servo Position", kicker.getPosition());
            telemetry.addData("Shooter Speed", shooter_speed);
            telemetry.addData("Battery", getBatteryVoltage());
            telemetry.update();

        }
    }
    
    
     /**
     * Returns the minimum voltage reported by any voltage sensor on the hardware map.
     * @return The lowest voltage reported.
     */
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
    
    public double getShooterSpeedFromVoltage() {
        double v = getBatteryVoltage();

        if (v > 14.0) return 0.83;
        if (v > 13.9) return 0.85;
        if (v >= 13.5) return 0.865;
        if (v >= 13.3) return 0.87;
        if (v >= 13.2) return 0.88;
        if (v >= 13.1) return 0.89;
        if (v >= 13.0) return 0.90;
        if (v >= 12.9) return 0.91;
        if (v >= 12.7) return 0.92;
        if (v > 12.5) return 0.94;
        if (v >= 12.0) return 0.95;

        return 1.0;
    }

    
    
    // ---------------- AUTO SHOOT LOGIC ----------------
    void runAutoShoot() {

        switch (shootState) {

            case SPIN_UP:
                double shooterSpeed = getShooterSpeedFromVoltage();
                leftshooter.setPower(shooterSpeed);
                rightshooter.setPower(shooterSpeed);
                if (shooterTimer.milliseconds() > 800) {
                    shooterTimer.reset();
                    shootState = AutoShootState.SHOOT_1;
                    runAutoShoot();
                }
                break;

            case SHOOT_1:
                kicker.setPosition(-0.8); // kicker up
                if (shooterTimer.milliseconds() > 800) {
                    kicker.setPosition(0.5); // kicker down
                    shooterTimer.reset();
                    shootState = AutoShootState.FEED_2;
                    runAutoShoot();
                }
                break;

            case FEED_2:
                belt.setPower(1);
                if (shooterTimer.milliseconds() > 800) {
                    belt.setPower(0);
                    intake.setPower(0);
                    shooterTimer.reset();
                    shootState = AutoShootState.SHOOT_2;
                    runAutoShoot();
                }
                break;

            case SHOOT_2:
                kicker.setPosition(-0.8);
                if (shooterTimer.milliseconds() > 600) {
                    kicker.setPosition(0.5);
                    shooterTimer.reset();
                }
                if (shooterTimer.milliseconds() > 300) {
                    shooterTimer.reset();
                    shootState = AutoShootState.FEED_3;
                    runAutoShoot();
                }
                break;

            case FEED_3:
                intake.setPower(1);
                belt.setPower(1);
                if (shooterTimer.milliseconds() > 1200) {
                    intake.setPower(0);
                    belt.setPower(0);
                    shooterTimer.reset();
                    shootState = AutoShootState.SHOOT_3;
                    runAutoShoot();
                }
                break;

            case SHOOT_3:
                kicker.setPosition(-0.8);
                if (shooterTimer.milliseconds() > 900) {
                    kicker.setPosition(0.5);
                    shootState = AutoShootState.DONE;
                    runAutoShoot();
                    
                }
                break;

            case DONE:
                stopAllShooter();
                autoShooting = false;
                shootState = AutoShootState.IDLE;
                break;

            default:
                break;
        }
    }

    // ---------------- SAFETY ----------------
    void stopAllShooter() {
        leftshooter.setPower(0);
        rightshooter.setPower(0);
        belt.setPower(0);
        intake.setPower(0);
    }

        
   
}
