package org.firstinspires.ftc.teamcode.latest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.limelightvision.limelightvisionlib.*;

@Autonomous(name="Auto_Limelight_Fused", group="Drive")
public class AutoLimelightFused extends LinearOpMode {

    // ---------------- HARDWARE ----------------
    DcMotorEx fl, fr, bl, br;
    DcMotorEx intake, belt, shooterLeft, shooterRight;
    Servo kicker;

    Limelight3A limelight;
    IMU imu;

    ElapsedTime stateTimer = new ElapsedTime();

    // ---------------- STATE ----------------
    enum AutoState {
        DRIVE_TO_SHOOT_VISION,
        SHOOT_ALIGN,
        SHOOT,
        COLLECT_ENCODER,
        DONE
    }
    AutoState state = AutoState.DRIVE_TO_SHOOT_VISION;

    boolean moveStarted = false;

    // ---------------- CONSTANTS ----------------
    static final double kP_DRIVE = 0.8;
    static final double kP_TURN  = 0.015;
    static final double POSITION_TOL = 0.05; // meters
    static final double HEADING_TOL  = 2.0;  // deg

    // Encoder
    static final double TICKS_PER_REV = 537.6;
    static final double WHEEL_DIAMETER_IN = 4.0;
    static final double TICKS_PER_INCH =
            TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_IN);

    // ---------------- POSE MEMORY ----------------
    double lastX = 0, lastY = 0, lastHeading = 0;
    boolean poseValid = false;

    @Override
    public void runOpMode() {

        initHardware();

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive() && state != AutoState.DONE) {

            LLResult ll = limelight.getLatestResult();
            boolean hasVision = ll != null && ll.isValid() && ll.getBotpose() != null;

            // --------- POSE RECOVERY ---------
            double robotX, robotY, robotHeading;

            if (hasVision) {
                Pose3D p = ll.getBotpose();
                robotX = p.getX();
                robotY = p.getY();
                robotHeading = p.getYaw();
                lastX = robotX;
                lastY = robotY;
                lastHeading = robotHeading;
                poseValid = true;
            } else if (poseValid) {
                robotX = lastX;
                robotY = lastY;
                robotHeading = getIMUHeading();
            } else {
                robotX = 0;
                robotY = 0;
                robotHeading = getIMUHeading();
            }

            switch (state) {

                // ================= FULL VISION DRIVE =================
                case DRIVE_TO_SHOOT_VISION:

                    if (hasVision) {
                        boolean arrived = visionGoToPose(
                                robotX, robotY, robotHeading,
                                1.35, 0.75, 90
                        );
                        if (arrived) state = AutoState.SHOOT_ALIGN;
                    } else {
                        encoderDrive(36, 0.6);
                        state = AutoState.COLLECT_ENCODER;
                    }
                    break;

                // ================= APRILTAG SHOOT ALIGN =================
                case SHOOT_ALIGN:

                    if (hasVision) {
                        double headingError = angleWrap(90 - robotHeading);
                        double turn = headingError * 0.02;
                        turn = Range.clip(turn, -0.3, 0.3);

                        fl.setPower(turn);
                        fr.setPower(-turn);
                        bl.setPower(turn);
                        br.setPower(-turn);

                        if (Math.abs(headingError) < 1.5) {
                            stopDrive();
                            stateTimer.reset();
                            state = AutoState.SHOOT;
                        }
                    }
                    break;

                // ================= SHOOT =================
                case SHOOT:
                    shooterLeft.setPower(1.0);
                    shooterRight.setPower(1.0);

                    if (stateTimer.milliseconds() > 800) {
                        kicker.setPosition(1);
                    }
                    if (stateTimer.milliseconds() > 1200) {
                        kicker.setPosition(0.5);
                        shooterLeft.setPower(0);
                        shooterRight.setPower(0);
                        state = AutoState.DONE;
                    }
                    break;

                // ================= ENCODER FALLBACK =================
                case COLLECT_ENCODER:
                    if (!motorsBusy()) {
                        stopDrive();
                        state = AutoState.DONE;
                    }
                    break;
            }

            telemetry.addData("State", state);
            telemetry.addData("Vision", hasVision);
            telemetry.update();
        }
    }

    // ================= VISION DRIVE =================
    private boolean visionGoToPose(double x, double y, double h,
                                   double tx, double ty, double th) {

        double ex = tx - x;
        double ey = ty - y;
        double eh = angleWrap(th - h);

        double rad = Math.toRadians(h);
        double forward = ex * Math.cos(rad) + ey * Math.sin(rad);
        double strafe  = -ex * Math.sin(rad) + ey * Math.cos(rad);

        double fp = Range.clip(forward * kP_DRIVE, -1, 1);
        double sp = Range.clip(strafe  * kP_DRIVE, -1, 1);
        double tp = Range.clip(eh * kP_TURN, -0.5, 0.5);

        mecanum(fp, sp, tp);

        return Math.hypot(ex, ey) < POSITION_TOL && Math.abs(eh) < HEADING_TOL;
    }

    // ================= MECANUM =================
    private void mecanum(double f, double s, double t) {
        double flp = f + s + t;
        double frp = f - s - t;
        double blp = f - s + t;
        double brp = f + s - t;

        double max = Math.max(1,
                Math.max(Math.abs(flp),
                Math.max(Math.abs(frp),
                Math.max(Math.abs(blp), Math.abs(brp)))));

        fl.setPower(flp / max);
        fr.setPower(frp / max);
        bl.setPower(blp / max);
        br.setPower(brp / max);
    }

    // ================= ENCODER =================
    private void encoderDrive(double inches, double power) {
        int ticks = (int)(inches * TICKS_PER_INCH);

        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        fl.setTargetPosition(fl.getCurrentPosition() + ticks);
        fr.setTargetPosition(fr.getCurrentPosition() + ticks);
        bl.setTargetPosition(bl.getCurrentPosition() + ticks);
        br.setTargetPosition(br.getCurrentPosition() + ticks);

        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }

    private boolean motorsBusy() {
        return fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy();
    }

    // ================= IMU =================
    private double getIMUHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    // ================= HELPERS =================
    private void stopDrive() {
        fl.setPower(0); fr.setPower(0); bl.setPower(0); br.setPower(0);
    }

    private double angleWrap(double a) {
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }

    // ================= INIT =================
    private void initHardware() {

        fl = hardwareMap.get(DcMotorEx.class, "leftfront");
        fr = hardwareMap.get(DcMotorEx.class, "rightfront");
        bl = hardwareMap.get(DcMotorEx.class, "leftrear");
        br = hardwareMap.get(DcMotorEx.class, "rightrear");

        shooterLeft  = hardwareMap.get(DcMotorEx.class, "leftshooter");
        shooterRight = hardwareMap.get(DcMotorEx.class, "rightshooter");
        kicker = hardwareMap.get(Servo.class, "ballkicker");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPipeline(0);
        limelight.setLEDMode(Limelight3A.LEDMode.ON);
        limelight.setCameraMode(Limelight3A.CameraMode.VISION);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        fr.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
    }
}
