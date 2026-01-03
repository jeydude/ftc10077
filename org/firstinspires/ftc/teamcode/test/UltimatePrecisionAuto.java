package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name="UltimatePrecisionAuto", group="Robot")
public class UltimatePrecisionAuto extends LinearOpMode {

    // --- Configuration ---
    // SET TO TRUE IF CAMERA IS FACING THE REAR OF THE ROBOT
    private static final boolean CAMERA_REVERSED = false; 
    private static final int TARGET_TAG_ID = 7; 

    // Hardware
    private DcMotorEx fl, fr, bl, br, intake, belt, shooterLeft, shooterRight;
    private IMU imu;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Constants
    private static final double TICKS_PER_INCH = 39.35;
    private static final double NOMINAL_VOLTAGE = 13.0;
    private static final double SHOOTER_DISTANCE_GAIN = 0.0068; 
    
    // PID Drive
    private double Kp = 0.045, Ki = 0.0001, Kd = 0.015;
    private double integralSum = 0, lastError = 0;
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        initializeHardware();
        initVision();

        while (!isStarted() && !isStopRequested()) {
            telemetryAprilTag();
            telemetry.update();
        }

        waitForStart();

        if (opModeIsActive()) {
            
            // 1. Initial Move
            driveWithPID(49, 4.0);
            
            // 2. Vision Alignment & First Shoot
            alignWithTag(TARGET_TAG_ID, 2.0);
            double range1 = getAprilTagRange(TARGET_TAG_ID);
            shootSequence(range1, 3.5); 

            // 3. Move to Collect Balls
            strafeInches(24, 0.6, 2.5);
            imuTurn(90, 2.0);
            
            intake.setPower(1.0);
            belt.setPower(0.5);
            driveWithPID(15, 2.0); 
            sleep(800);
            intake.setPower(0);
            
            // 4. Return to Shooting Position
            driveWithPID(-15, 2.0); 
            imuTurn(0, 2.0);        
            strafeInches(-24, 0.6, 2.5); 

            // 5. FINAL VISION ALIGNMENT
            // This fixes any errors caused by the collection movement
            alignWithTag(TARGET_TAG_ID, 2.0);

            // 6. Final Decoded Shoot
            double range2 = getAprilTagRange(TARGET_TAG_ID);
            shootSequence(range2, 4.0);
            
            stopRobot();
        }
    }

    /**
     * Uses Webcam to point exactly at the target.
     * Handles Front/Back camera mounting automatically.
     */
    public void alignWithTag(int targetId, double timeout) {
        timer.reset();
        double STRAFE_P = 0.025; 
        double TURN_P   = 0.03;  
        
        while (opModeIsActive() && timer.seconds() < timeout) {
            AprilTagDetection targetTag = null;
            List<AprilTagDetection> detections = aprilTag.getDetections();
            for (AprilTagDetection d : detections) {
                if (d.id == targetId && d.metadata != null) { targetTag = d; break; }
            }

            if (targetTag != null) {
                // If camera is on back, we must flip the error signs
                double multiplier = CAMERA_REVERSED ? -1.0 : 1.0;
                
                double bearingError = targetTag.ftcPose.bearing * multiplier;
                double yawError     = targetTag.ftcPose.yaw * multiplier;

                double strafePower = Range.clip(bearingError * STRAFE_P, -0.4, 0.4);
                double turnPower   = Range.clip(yawError * TURN_P, -0.3, 0.3);

                // Mecanum logic for centering
                fl.setPower(-strafePower - turnPower);
                fr.setPower(strafePower + turnPower);
                bl.setPower(strafePower - turnPower);
                br.setPower(-strafePower + turnPower);

                if (Math.abs(bearingError) < 1.0 && Math.abs(yawError) < 1.0) break;
            } else {
                applyDrivePower(0, 0, 0, 0);
            }
        }
        applyDrivePower(0, 0, 0, 0);
    }

    private void shootSequence(double range, double duration) {
        if (range <= 0) range = 50.0; 
        double voltageFactor = NOMINAL_VOLTAGE / getVoltage();
        double finalPower = Range.clip(((range * SHOOTER_DISTANCE_GAIN) + 0.32) * voltageFactor, 0, 1.0);

        shooterLeft.setPower(finalPower);
        shooterRight.setPower(finalPower);
        sleep(1000); 
        belt.setPower(0.8); 
        sleep((long)(duration * 1000));
        stopShooter();
    }

    // --- Movement & PID ---
    public void driveWithPID(double targetInches, double timeout) {
        double targetTicks = targetInches * TICKS_PER_INCH;
        resetDriveEncoders();
        timer.reset();
        integralSum = 0; lastError = 0;

        while (opModeIsActive() && timer.seconds() < timeout) {
            double error = targetTicks - fl.getCurrentPosition();
            integralSum += error * timer.seconds();
            double derivative = (error - lastError) / timer.seconds();
            lastError = error;
            double output = (error * Kp) + (integralSum * Ki) + (derivative * Kd);
            applyDrivePower(Range.clip(output, -0.7, 0.7));
            if (Math.abs(error) < 12) break;
        }
        applyDrivePower(0);
    }

    public void imuTurn(double targetAngle, double timeout) {
        timer.reset();
        while (opModeIsActive() && timer.seconds() < timeout) {
            double error = targetAngle - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double turnPower = Range.clip(error * 0.025, -0.6, 0.6);
            fl.setPower(-turnPower); bl.setPower(-turnPower);
            fr.setPower(turnPower); br.setPower(turnPower);
            if (Math.abs(error) < 1.0) break;
        }
        applyDrivePower(0);
    }

    // --- Hardware & Utility ---
    private void initializeHardware() {
        fl = hardwareMap.get(DcMotorEx.class, "leftfront");
        fr = hardwareMap.get(DcMotorEx.class, "rightfront");
        bl = hardwareMap.get(DcMotorEx.class, "leftrear");
        br = hardwareMap.get(DcMotorEx.class, "rightrear");
        shooterLeft = hardwareMap.get(DcMotorEx.class, "leftshooter");
        shooterRight = hardwareMap.get(DcMotorEx.class, "rightshooter");
        intake = hardwareMap.get(DcMotorEx.class, "frontintake");
        belt = hardwareMap.get(DcMotorEx.class, "belt");

        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
        shooterLeft.setDirection(DcMotor.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
    }

    private void initVision() {
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag).build();
    }

    public void strafeInches(double inches, double power, double timeout) {
        int target = (int)(inches * TICKS_PER_INCH * 1.15);
        fl.setTargetPosition(fl.getCurrentPosition() + target);
        fr.setTargetPosition(fr.getCurrentPosition() - target);
        bl.setTargetPosition(bl.getCurrentPosition() - target);
        br.setTargetPosition(br.getCurrentPosition() + target);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        applyDrivePower(power);
        timer.reset();
        while (opModeIsActive() && timer.seconds() < timeout && fl.isBusy()) { }
        applyDrivePower(0);
        resetDriveEncoders();
    }

    private void applyDrivePower(double p) { fl.setPower(p); fr.setPower(p); bl.setPower(p); br.setPower(p); }
    private void applyDrivePower(double p1, double p2, double p3, double p4) { fl.setPower(p1); fr.setPower(p2); bl.setPower(p3); br.setPower(p4); }
    
    private void setMode(DcMotor.RunMode mode) { fl.setMode(mode); fr.setMode(mode); bl.setMode(mode); br.setMode(mode); }

    private void resetDriveEncoders() {
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private double getVoltage() {
        double result = 12.0;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            if (sensor.getVoltage() > 0) result = sensor.getVoltage();
        }
        return result;
    }

    private double getAprilTagRange(int targetId) {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection d : detections) {
            if (d.id == targetId && d.metadata != null) return d.ftcPose.range;
        }
        return -1;
    }

    private void telemetryAprilTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection d : detections) {
            if (d.metadata != null) {
                telemetry.addLine(String.format("ID %d: Range %.2f, Bearing %.2f", d.id, d.ftcPose.range, d.ftcPose.bearing));
            }
        }
    }

    private void stopShooter() { shooterLeft.setPower(0); shooterRight.setPower(0); belt.setPower(0); }
    private void stopRobot() { applyDrivePower(0); stopShooter(); }
}