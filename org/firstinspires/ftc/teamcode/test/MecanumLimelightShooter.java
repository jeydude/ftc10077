package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

@TeleOp(name = "Mecanum + Limelight Shooter (goBILDA)", group = "Drive")
public class MecanumLimelightShooter extends LinearOpMode {

    // ================= MOTORS =================
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx shooter;

    // ================= LIMELIGHT =================
    private NetworkTable limelight;
    private NetworkTableEntry botpose;

    // ================= CONSTANTS =================

    // Encoder
    static final double TICKS_PER_REV = 537.7;

    // Drive motors
    static final double DRIVE_MAX_RPM = 312.0;

    // Shooter motor
    static final double SHOOTER_MAX_RPM = 1620.0;

    // Shooter physics (TUNE THESE)
    static final double SHOOTER_ANGLE_DEG = 40.0;
    static final double WHEEL_RADIUS = 0.05; // meters

    // AprilTag target location (FIELD COORDINATES)
    static final double TARGET_X = 1.35; // meters
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

        telemetry.addLine("Ready - goBILDA drive + shooter");
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
            // LIMELIGHT SHOOTER CONTROL
            // Hold RIGHT TRIGGER to spin shooter
            // ==================================================
            double[] pose = botpose.getDoubleArray(new double[6]);

            if (pose.length >= 2 && gamepad1.right_trigger > 0.5) {

                double robotX = pose[0];
                double robotY = pose[1];

                double dx = TARGET_X - robotX;
                double dy = TARGET_Y - robotY;
                double distance = Math.hypot(dx, dy);

                // Projectile math (baseline)
                double angleRad = Math.toRadians(SHOOTER_ANGLE_DEG);
                double exitVelocity = Math.sqrt(
                        (distance * 9.81) / Math.sin(2 * angleRad)
                );

                // Convert to RPM
                double rpm = (exitVelocity / (2 * Math.PI * WHEEL_RADIUS)) * 60.0;

                // Clamp to shooter capability
                rpm = Math.min(rpm, SHOOTER_MAX_RPM);

                // Convert RPM -> ticks/sec
                double ticksPerSecond = rpm * TICKS_PER_REV / 60.0;

                shooter.setVelocity(ticksPerSecond);

                telemetry.addData("Distance (m)", "%.2f", distance);
                telemetry.addData("Shooter RPM", "%.0f", rpm);

            } else {
                shooter.setVelocity(0);
            }

            telemetry.update();
        }
    }
}
