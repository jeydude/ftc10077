package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name = "TeleOp", group = "Drive")
public class TeleOpDrive extends OpMode {

    // Drive motors
    private DcMotorEx fl, fr, bl, br;
    private DcMotorEx intake, belt, shooterLeft, shooterRight;
    private Servo kicker;

    // Kicker positions
    private static final double KICKER_REST = 0.5;
    private static final double KICKER_KICK = -0.8;

    private boolean shooterOn = false;
    private boolean kickPressed = false;

    private ShootState shootState = ShootState.IDLE;
    private ElapsedTime shootTimer = new ElapsedTime();

    private enum ShootState {
        IDLE,
        SPIN_UP,
        KICK_1,
        RESET_1,
        FEED_2,
        KICK_2,
        RESET_2,
        FEED_3,
        KICK_3,
        RESET_3,
        DONE
    }

    @Override
    public void init() {

        fl = hardwareMap.get(DcMotorEx.class, "frontLeft");
        fr = hardwareMap.get(DcMotorEx.class, "frontRight");
        bl = hardwareMap.get(DcMotorEx.class, "backLeft");
        br = hardwareMap.get(DcMotorEx.class, "backRight");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        belt = hardwareMap.get(DcMotorEx.class, "belt");

        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        kicker = hardwareMap.get(Servo.class, "kicker");

        // Motor directions
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
        shooterRight.setDirection(DcMotor.Direction.REVERSE);

        // Braking
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        belt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        kicker.setPosition(KICKER_REST);

        telemetry.addLine("TeleOp Ready");
        telemetry.update();
    }

    @Override
    public void loop() {

        /* -------------------- DRIVE -------------------- */
        double y = -gamepad1.left_stick_y;   // forward/back
        double x = gamepad1.left_stick_x;    // strafe
        double rx = gamepad1.right_stick_x;  // turn

        double slow = gamepad1.right_bumper ? 0.4 : 1.0;

        double flPower = (y + x + rx) * slow;
        double frPower = (y - x - rx) * slow;
        double blPower = (y - x + rx) * slow;
        double brPower = (y + x - rx) * slow;

        double max = Math.max(1.0,
                Math.max(Math.abs(flPower),
                Math.max(Math.abs(frPower),
                Math.max(Math.abs(blPower), Math.abs(brPower)))));

        fl.setPower(flPower / max);
        fr.setPower(frPower / max);
        bl.setPower(blPower / max);
        br.setPower(brPower / max);

        /* -------------------- INTAKE -------------------- */
        if (gamepad2.right_trigger > 0.1) {
            intake.setPower(gamepad2.right_trigger);
        } else if (gamepad2.left_trigger > 0.1) {
            intake.setPower(-gamepad2.left_trigger);
        } else {
            intake.setPower(0);
        }

        /* -------------------- BELT -------------------- */
        if (gamepad2.a) {
            belt.setPower(0.5);
        } else if (gamepad2.b) {
            belt.setPower(-0.5);
        } else {
            belt.setPower(0);
        }

        /* -------------------- SHOOTER -------------------- */
        if (gamepad2.x && !shooterOn) {
            shooterOn(calculateShooterPower());
            shooterOn = true;
        }

        if (gamepad2.y) {
            shooterOff();
            shooterOn = false;
        }

        /* -------------------- KICKER -------------------- */
        if (gamepad2.right_bumper && !kickPressed) {
            kicker.setPosition(KICKER_KICK);
            kickPressed = true;
        } else if (!gamepad2.right_bumper && kickPressed) {
            kicker.setPosition(KICKER_REST);
            kickPressed = false;
        }

        // Cancel 3-ball macro
        if (gamepad2.dpad_down && shootState != ShootState.IDLE) {
            shootState = ShootState.IDLE;
            shooterOff();
        }

        /* -------------------- SHOOT 3 -------------------- */
        // Start 3-ball macro
        if (gamepad2.dpad_up && shootState == ShootState.IDLE) {
            shootState = ShootState.SPIN_UP;
            shootTimer.reset();
            updateShoot3();
        }


        telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
        telemetry.addData("Voltage", getBatteryVoltage());
        telemetry.update();
    }

    /* -------------------- HELPERS -------------------- */

    private void shooterOn(double power) {
        shooterLeft.setPower(power);
        shooterRight.setPower(power);
    }

    private void shooterOff() {
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
    }

    private double getBatteryVoltage() {
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > 0) return v;
        }
        return 12.0;
    }

    private double calculateShooterPower() {
        double voltage = getBatteryVoltage();
        double minVoltage = 12.0;
        double maxVoltage = 13.8;
        double minPower = 0.85;
        double maxPower = 1.0;

        double power = maxPower -
                ((voltage - minVoltage) / (maxVoltage - minVoltage))
                * (maxPower - minPower);

        return Math.min(maxPower, Math.max(minPower, power));
    }

    private void updateShoot3() {

        switch (shootState) {

            case IDLE:
                // Do nothing
                break;

            case SPIN_UP:
                shooterOn(calculateShooterPower());
                if (shootTimer.milliseconds() > 1200) {
                    shootState = ShootState.KICK_1;
                    shootTimer.reset();
                    updateShoot3();
                }
                break;

            case KICK_1:
                kicker.setPosition(KICKER_KICK);
                if (shootTimer.milliseconds() > 180) {
                    shootState = ShootState.RESET_1;
                    shootTimer.reset();
                    updateShoot3();
                }
                break;

            case RESET_1:
                // 1st ball has been kicked, bring in next ball while resetting kicker
                // Reverse belt briefly to ensure ball is clear
                belt.setPower(-0.8);
                if (shootTimer.milliseconds() > 150) {
                    shootTimer.reset();
                }
                kicker.setPosition(KICKER_REST);
                belt.setPower(0);
                if (shootTimer.milliseconds() > 150) {
                    shootState = ShootState.FEED_2;
                    shootTimer.reset();
                    updateShoot3();
                }
                break;

            case FEED_2:
                belt.setPower(0.8);
                if (shootTimer.milliseconds() > 500) {
                    belt.setPower(0);
                    intake.setPower(0);
                    shootState = ShootState.KICK_2;
                    shootTimer.reset();
                    updateShoot3();
                }
                break;

            case KICK_2:
                kicker.setPosition(KICKER_KICK);
                if (shootTimer.milliseconds() > 180) {
                    shootState = ShootState.RESET_2;
                    shootTimer.reset();
                    updateShoot3();
                }
                break;

            case RESET_2:
                belt.setPower(-0.8);
                if (shootTimer.milliseconds() > 150) {
                    shootTimer.reset();
                }
                kicker.setPosition(KICKER_REST);
                belt.setPower(0);
                if (shootTimer.milliseconds() > 150) {
                    shootState = ShootState.FEED_3;
                    shootTimer.reset();
                    updateShoot3();
                }
                break;

            case FEED_3:
                belt.setPower(0.8);
                intake.setPower(0.5);
                if (shootTimer.milliseconds() > 600) {
                    belt.setPower(0);
                    intake.setPower(0);
                    shootState = ShootState.KICK_3;
                    shootTimer.reset();
                    updateShoot3();
                }
                break;

            case KICK_3:
                kicker.setPosition(KICKER_KICK);
                if (shootTimer.milliseconds() > 180) {
                    shootState = ShootState.RESET_3;
                    shootTimer.reset();
                    updateShoot3();
                }
                break;

            case RESET_3:
                kicker.setPosition(KICKER_REST);
                if (shootTimer.milliseconds() > 150) {
                    shootState = ShootState.DONE;
                    updateShoot3(); 
                }
                break;

            case DONE:
                shooterOff();              // remove if you want shooter to stay on
                shootState = ShootState.IDLE;
                break;
        }
    }

}