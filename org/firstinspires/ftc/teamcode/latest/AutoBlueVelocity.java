package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutoBLUE_Velocity", group="Autonomous")
public class AutoBlueVelocity extends LinearOpMode {

    // Drive motors
    private DcMotorEx frontLeft = null;
    private DcMotorEx frontRight = null;
    private DcMotorEx backLeft = null;
    private DcMotorEx backRight = null;

    // Other motors/servos
    private DcMotor intake = null;
    private DcMotor leftshooter = null;
    private DcMotor rightshooter = null;
    private DcMotor belt = null;
    private Servo kicker = null;

    private ElapsedTime runtime = new ElapsedTime();

    // Constants for GoBilda 312 RPM (or your 5203 motor)
    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    private double shooter_speed = 1.0;

    @Override
    public void runOpMode() {

        // Initialize hardware
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftfront");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightfront");
        backLeft = hardwareMap.get(DcMotorEx.class, "leftrear");
        backRight = hardwareMap.get(DcMotorEx.class, "rightrear");
        intake = hardwareMap.get(DcMotor.class, "frontintake");
        leftshooter = hardwareMap.get(DcMotor.class, "leftshooter");
        rightshooter = hardwareMap.get(DcMotor.class, "rightshooter");
        belt = hardwareMap.get(DcMotor.class, "belt");
        kicker = hardwareMap.get(Servo.class, "ballkicker");

        // Set directions
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        leftshooter.setDirection(DcMotor.Direction.REVERSE);
        rightshooter.setDirection(DcMotor.Direction.FORWARD);

        // Set all drive motors to RUN_USING_ENCODER
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Brake behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftshooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightshooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Shooter speed adjustment
        shooter_speed = getShooterSpeedFromVoltage();
        leftshooter.setPower(shooter_speed);
        rightshooter.setPower(shooter_speed);

        // Drive forward 40 inches at 70% max RPM
        driveForwardVelocity(40, 0.7 * 312);

        telemetry.addData("Shooter Speed", shooter_speed);
        telemetry.addData("Battery", getBatteryVoltage());
        telemetry.update();

        // Shoot balls
        shoot_all();

        leftshooter.setPower(0);
        rightshooter.setPower(0);

        shootnextballs();

        telemetry.addData("Status", "Autonomous Complete");
        telemetry.update();
    }

    // ---------------- DRIVE FUNCTIONS ----------------

    private void driveForwardVelocity(double inches, double rpm) {
        double targetTicks = inches * COUNTS_PER_INCH;

        int flStart = frontLeft.getCurrentPosition();
        int frStart = frontRight.getCurrentPosition();
        int blStart = backLeft.getCurrentPosition();
        int brStart = backRight.getCurrentPosition();

        double ticksPerSec = rpm * COUNTS_PER_MOTOR_REV / 60.0;

        frontLeft.setVelocity(ticksPerSec);
        frontRight.setVelocity(ticksPerSec);
        backLeft.setVelocity(ticksPerSec);
        backRight.setVelocity(ticksPerSec);

        while (opModeIsActive() &&
                Math.abs(frontLeft.getCurrentPosition() - flStart) < targetTicks &&
                Math.abs(frontRight.getCurrentPosition() - frStart) < targetTicks &&
                Math.abs(backLeft.getCurrentPosition() - blStart) < targetTicks &&
                Math.abs(backRight.getCurrentPosition() - brStart) < targetTicks) {
            telemetry.addData("FL", frontLeft.getCurrentPosition());
            telemetry.addData("FR", frontRight.getCurrentPosition());
            telemetry.update();
        }

        stopAllDriveMotors();
    }

    private void strafeVelocity(double inches, double rpm, int direction) {
        double targetTicks = inches * COUNTS_PER_INCH;

        int flStart = frontLeft.getCurrentPosition();
        int frStart = frontRight.getCurrentPosition();
        int blStart = backLeft.getCurrentPosition();
        int brStart = backRight.getCurrentPosition();

        double ticksPerSec = rpm * COUNTS_PER_MOTOR_REV / 60.0;

        // Direction: 1 = left, 0 = right
        double fl = direction == 1 ? -ticksPerSec : ticksPerSec;
        double fr = direction == 1 ? ticksPerSec : -ticksPerSec;
        double bl = direction == 1 ? ticksPerSec : -ticksPerSec;
        double br = direction == 1 ? -ticksPerSec : ticksPerSec;

        frontLeft.setVelocity(fl);
        frontRight.setVelocity(fr);
        backLeft.setVelocity(bl);
        backRight.setVelocity(br);

        while (opModeIsActive() &&
                Math.abs(frontLeft.getCurrentPosition() - flStart) < targetTicks &&
                Math.abs(frontRight.getCurrentPosition() - frStart) < targetTicks &&
                Math.abs(backLeft.getCurrentPosition() - blStart) < targetTicks &&
                Math.abs(backRight.getCurrentPosition() - brStart) < targetTicks) {
            telemetry.addData("FL", frontLeft.getCurrentPosition());
            telemetry.addData("FR", frontRight.getCurrentPosition());
            telemetry.update();
        }

        stopAllDriveMotors();
    }

    private void turnVelocity(double degrees, double rpm) {
        final double COUNTS_PER_DEGREE = 9.7; // calibrate for your robot
        double targetTicks = degrees * COUNTS_PER_DEGREE;

        int flStart = frontLeft.getCurrentPosition();
        int frStart = frontRight.getCurrentPosition();
        int blStart = backLeft.getCurrentPosition();
        int brStart = backRight.getCurrentPosition();

        double ticksPerSec = rpm * COUNTS_PER_MOTOR_REV / 60.0;

        frontLeft.setVelocity(ticksPerSec);
        backLeft.setVelocity(ticksPerSec);
        frontRight.setVelocity(-ticksPerSec);
        backRight.setVelocity(-ticksPerSec);

        while (opModeIsActive() &&
                Math.abs(frontLeft.getCurrentPosition() - flStart) < targetTicks &&
                Math.abs(frontRight.getCurrentPosition() - frStart) < targetTicks) {
            telemetry.addData("FL", frontLeft.getCurrentPosition());
            telemetry.addData("FR", frontRight.getCurrentPosition());
            telemetry.update();
        }

        stopAllDriveMotors();
    }

    private void stopAllDriveMotors() {
        frontLeft.setVelocity(0);
        frontRight.setVelocity(0);
        backLeft.setVelocity(0);
        backRight.setVelocity(0);
    }

    // ---------------- SHOOTER & INTAKE ----------------

    private void shoot_all() {
        shoot(1);
        shoot(2);
        shoot(3);
    }

    private void shoot(int number) {
        if (number == 2 || number == 5) {
            belt.setPower(1);
            sleep(500);
        } else if (number == 3 || number == 6) {
            intake.setPower(1);
            sleep(400);
            intake.setPower(0);
            belt.setPower(1);
            sleep(1200);
        }
        belt.setPower(0);

        kicker.setPosition(-0.8);
        sleep(1200);
        kicker.setPosition(0.5);
        sleep(500);
    }

    private void shootnextballs() {
        driveForwardVelocity(5, 0.7 * 312);
        turnVelocity(-130, 0.7 * 312);
        strafeVelocity(7, 0.6 * 312, 1);

        intake.setPower(1);
        belt.setPower(0.5);
        driveForwardVelocity(38, 0.3 * 312);
        sleep(250);
        belt.setPower(0);
        intake.setPower(0);

        bringballdown();

        driveForwardVelocity(-30, 0.6 * 312);
        sleep(300);
        turnVelocity(145, 0.7 * 312);
        strafeVelocity(15, 0.6 * 312, 1);

        shooter_speed = getShooterSpeedFromVoltage();
        leftshooter.setPower(shooter_speed);
        rightshooter.setPower(shooter_speed);
        shoot(4);
        shoot(5);
        shoot(6);
        leftshooter.setPower(0);
        rightshooter.setPower(0);
        strafeVelocity(-25, 0.6 * 312, 1);
    }

    private void bringballdown() {
        leftshooter.setPower(-0.5);
        rightshooter.setPower(-0.5);
        intake.setPower(-0.5);
        belt.setPower(-0.8);
        sleep(250);
        belt.setPower(0);
        intake.setPower(0);
        leftshooter.setPower(0);
        rightshooter.setPower(0);
    }

    // ---------------- HELPER FUNCTIONS ----------------

    private double getShooterSpeedFromVoltage() {
        double v = getBatteryVoltage();
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

    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) result = Math.min(result, voltage);
        }
        return result;
    }
}
