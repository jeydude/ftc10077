package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

@TeleOp(name = "Mecanum + Limelight Shooter (Geared)", group = "Drive")
public class MecanumLimelightShooter extends LinearOpMode {

    // ================= MOTORS =================
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx shooter;

    // ================= LIMELIGHT =================
    private NetworkTable limelight;
    private NetworkTableEntry botpose;

    // ================= CONSTANTS =================

    // Shooter Encoder
    static final double MOTOR_TICKS_PER_REV = 103.8;

    // Shooter gearing
    static final double SHOOTER_GEAR_RATIO = 2.0; // 40T / 20T

    // Shooter limits
    static final double SHOOTER_MOTOR_MAX_RPM = 1620.0;
    static final double SHOOTER_WHEEL_MAX_RPM = SHOOTER_MOTOR_MAX_RPM / SHOOTER_GEAR_RATIO;

    // Shooter physics (baseline)
    static final double SHOOTER_ANGLE_DEG = 40.0;
    static final double WHEEL_RADIUS = 0.05; // meters

    // AprilTag target location (meters)
    static final double TARGET_X = 1.35;
    static final double TARGET_Y = 3.90;

    @Override
    public void runOpMode() {

        // ================= HARDWARE MAP =================
        frontLeft  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight  = hardwareMap.get(DcMotorEx.class, "backRight");

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        // ================= MOTOR DIRECTIONS =================
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // ================= ENCODERS =================
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ================= BRAKES =================
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ================= LIMELIGHT =================
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
        botpose = limelight.getEntry("botpose");

        telemetry.addLine("Ready - Shooter 2:1 Geared");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ==================================================
            // MECANUM DRIVE
            // ==================================================
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;

            double fl = y + x + rx;
            double bl = y - x + rx;
            double fr = y - x - rx;
            double br = y + x - rx;

            double max = Math.max(
                    Math.max(Math.abs(fl), Math.abs(bl)),
                    Math.max(Math.abs(fr), Math.abs(br))
            );

            if (max > 1.0) {
                fl /= max;
                bl /= max;
                fr /= max;
                br /= max;
            }

            frontLeft.setPower(fl);
            backLeft.setPower(bl);
            frontRight.setPower(fr);
            backRight.setPower(br);

            // ==================================================
            // LIMELIGHT SHOOTER
            // Hold RIGHT TRIGGER to spin up
            // ==================================================
            double[] pose = botpose.getDoubleArray(new double[6]);

            if (pose.length >= 2 && gamepad1.right_trigger > 0.5) {

                double robotX = pose[0];
                double robotY = pose[1];

                double distance = Math.hypot(
                        TARGET_X - robotX,
                        TARGET_Y - robotY
                );

                // Projectile baseline
                // Projectile baseline (replace with empirical tuning later)
                double angleRad = Math.toRadians(SHOOTER_ANGLE_DEG);
                targetWheelRPM =
                        (Math.sqrt((distance * 9.81) / Math.sin(2 * angleRad))
                                / (2 * Math.PI * WHEEL_RADIUS)) * 60.0;

                // Clamp to physical limit
                targetWheelRPM = Math.min(targetWheelRPM, SHOOTER_WHEEL_MAX_RPM);

                // Convert wheel RPM -> motor RPM
                double motorRPM = targetWheelRPM * SHOOTER_GEAR_RATIO;

                // Convert motor RPM -> ticks/sec
                double ticksPerSecond =
                        motorRPM * MOTOR_TICKS_PER_REV / 60.0;

                shooter.setVelocity(ticksPerSecond);

                telemetry.addData("Distance (m)", "%.2f", distance);
                telemetry.addData("Target Wheel RPM", "%.0f", targetWheelRPM);
                telemetry.addData("Motor RPM", "%.0f", motorRPM);
            } else {
                shooter.setVelocity(0);
            }

            telemetry.update();
        }
    }
}
