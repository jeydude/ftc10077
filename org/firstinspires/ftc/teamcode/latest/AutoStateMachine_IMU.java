package org.firstinspires.ftc.teamcode.latest;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.*;

@Autonomous(name = "AutoStateMachine_IMU", group = "Drive")
public class AutoStateMachine_IMU extends LinearOpMode {

    // -------------------- HARDWARE --------------------
    private DcMotorEx fl, fr, bl, br;
    private DcMotorEx intake, belt, shooterLeft, shooterRight;
    private Servo kicker;
    private BNO055IMU imu;

    private ElapsedTime stateTimer = new ElapsedTime();

    // -------------------- STATE MACHINE --------------------
    private enum AutoState {
        START,
        BALL1_KICK,
        BALL2_FEED,
        BALL2_KICK,
        BALL3_FEED,
        BALL3_KICK,

        MOVE_FORWARD_STACK,
        TURN_TO_STACK,
        STRAFE_TO_STACK,

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

        PARK,
        DONE
    }

    private AutoState state = AutoState.START;
    private boolean moveStarted = false;

    // -------------------- CONSTANTS --------------------
    private static final double TICKS_PER_REV = 537.6;
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double GEAR_RATIO = 1.0;
    private static final double CORRECTION_VALUE = 0.96;

    private static final double TICKS_PER_INCH =
            (TICKS_PER_REV * GEAR_RATIO) /
            (Math.PI * WHEEL_DIAMETER_INCHES) * CORRECTION_VALUE;

    private static final double STRAFE_MULTIPLIER = 1.03;

    private static final double KICKER_REST = 0.5;
    private static final double KICKER_KICK = -1.0;

    // Encoder targets
    private int flTarget, frTarget, blTarget, brTarget;

    // IMU turn target
    private double targetHeading;

    // -------------------- OPMODE --------------------
    @Override
    public void runOpMode() {

        initHardware(hardwareMap);
        initIMU(hardwareMap);

        telemetry.addLine("IMU Auto Ready");
        telemetry.update();

        waitForStart();
        stateTimer.reset();

        while (opModeIsActive() && state != AutoState.DONE) {

            switch (state) {

                case START:
                    shooterOn(1.0);
                    driveFast(44, 0.85);
                    if (encoderMoveDone()) {
                        stopDrive();
                        moveStarted = false;
                        state = AutoState.BALL1_KICK;
                        stateTimer.reset();
                    }
                    break;

                case BALL1_KICK:
                    runKicker();
                    if (stateTimer.milliseconds() > 1200) {
                        state = AutoState.BALL2_FEED;
                        stateTimer.reset();
                    }
                    break;

                case BALL2_FEED:
                    beltOn(0.8);
                    if (stateTimer.milliseconds() > 800) {
                        beltOff();
                        state = AutoState.BALL2_KICK;
                        stateTimer.reset();
                    }
                    break;

                case BALL2_KICK:
                    runKicker();
                    if (stateTimer.milliseconds() > 1200) {
                        state = AutoState.BALL3_FEED;
                        stateTimer.reset();
                    }
                    break;

                case BALL3_FEED:
                    intakeOn(1.0);
                    beltOn(0.8);
                    if (stateTimer.milliseconds() > 800) {
                        intakeOff();
                        beltOff();
                        state = AutoState.BALL3_KICK;
                        stateTimer.reset();
                    }
                    break;

                case BALL3_KICK:
                    runKicker();
                    if (stateTimer.milliseconds() > 1200) {
                        shooterOff();
                        state = AutoState.MOVE_FORWARD_STACK;
                    }
                    break;

                case MOVE_FORWARD_STACK:
                    driveFast(5, 0.7);
                    if (encoderMoveDone()) {
                        stopDrive();
                        moveStarted = false;
                        state = AutoState.TURN_TO_STACK;
                        startIMUTurn(-120);
                    }
                    break;

                case TURN_TO_STACK:
                    if (imuTurnDone(0.7)) {
                        stopDrive();
                        state = AutoState.STRAFE_TO_STACK;
                        moveStarted = false;
                    }
                    break;

                case STRAFE_TO_STACK:
                    strafeFast(-4.5, 0.75);
                    if (encoderMoveDone()) {
                        stopDrive();
                        moveStarted = false;
                        state = AutoState.COLLECT_BALLS;
                    }
                    break;

                case COLLECT_BALLS:
                    intakeOn(1.0);
                    beltOn(0.5);
                    driveFast(40, 0.85);
                    if (encoderMoveDone()) {
                        stopDrive();
                        intakeOff();
                        beltOff();
                        moveStarted = false;
                        state = AutoState.ADJUST_BALLS;
                        stateTimer.reset();
                    }
                    break;

                case ADJUST_BALLS:
                    shooterOn(-0.5);
                    intakeOn(-0.5);
                    beltOn(-0.8);
                    if (stateTimer.milliseconds() > 250) {
                        shooterOff();
                        intakeOff();
                        beltOff();
                        state = AutoState.DRIVE_BACK_TO_SHOOT;
                    }
                    break;

                case DRIVE_BACK_TO_SHOOT:
                    driveFast(-40, 0.9);
                    if (encoderMoveDone()) {
                        stopDrive();
                        moveStarted = false;
                        state = AutoState.STRAFE_BACK_TO_SHOOT;
                    }
                    break;

                case STRAFE_BACK_TO_SHOOT:
                    strafeFast(4.5, 0.75);
                    if (encoderMoveDone()) {
                        stopDrive();
                        moveStarted = false;
                        startIMUTurn(120);
                        state = AutoState.TURN_BACK_TO_SHOOT;
                    }
                    break;

                case TURN_BACK_TO_SHOOT:
                    shooterOn(1.0);
                    if (imuTurnDone(0.7)) {
                        stopDrive();
                        state = AutoState.BALL4_KICK;
                        stateTimer.reset();
                    }
                    break;

                case BALL4_KICK:
                    runKicker();
                    if (stateTimer.milliseconds() > 1200) {
                        state = AutoState.BALL5_FEED;
                        stateTimer.reset();
                    }
                    break;

                case BALL5_FEED:
                    beltOn(0.8);
                    if (stateTimer.milliseconds() > 800) {
                        beltOff();
                        state = AutoState.BALL5_KICK;
                        stateTimer.reset();
                    }
                    break;

                case BALL5_KICK:
                    runKicker();
                    if (stateTimer.milliseconds() > 1200) {
                        state = AutoState.BALL6_FEED;
                        stateTimer.reset();
                    }
                    break;

                case BALL6_FEED:
                    intakeOn(1.0);
                    beltOn(0.8);
                    if (stateTimer.milliseconds() > 800) {
                        intakeOff();
                        beltOff();
                        state = AutoState.BALL6_KICK;
                        stateTimer.reset();
                    }
                    break;

                case BALL6_KICK:
                    runKicker();
                    if (stateTimer.milliseconds() > 1200) {
                        shooterOff();
                        state = AutoState.PARK;
                    }
                    break;

                case PARK:
                    strafeFast(20, 1.0);
                    if (encoderMoveDone()) {
                        stopDrive();
                        state = AutoState.DONE;
                    }
                    break;
            }

            telemetry.addData("State", state);
            telemetry.addData("Heading", getHeading());
            telemetry.update();
        }
    }

    // -------------------- IMU --------------------
    private void initIMU(HardwareMap hw) {
        imu = hw.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters p = new BNO055IMU.Parameters();
        p.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        p.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        p.loggingEnabled = false;

        imu.initialize(p);
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(20);
        }
    }

    private double getHeading() {
        Orientation o = imu.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES);
        return o.firstAngle;
    }

    private void startIMUTurn(double degrees) {
        targetHeading = normalizeAngle(getHeading() + degrees);
    }

    private boolean imuTurnDone(double power) {
        double error = normalizeAngle(targetHeading - getHeading());

        if (Math.abs(error) < 2.0) {
            return true;
        }

        double turnPower = Math.copySign(
                Math.max(0.25, Math.min(power, Math.abs(error) / 30.0)),
                error);

        fl.setPower(turnPower);
        bl.setPower(turnPower);
        fr.setPower(-turnPower);
        br.setPower(-turnPower);

        return false;
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    // -------------------- FAST ENCODER DRIVE --------------------
    private void startMove(int flT, int frT, int blT, int brT, double p) {

        flTarget = fl.getCurrentPosition() + flT;
        frTarget = fr.getCurrentPosition() + frT;
        blTarget = bl.getCurrentPosition() + blT;
        brTarget = br.getCurrentPosition() + brT;

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        fl.setPower(Math.signum(flT) * p);
        fr.setPower(Math.signum(frT) * p);
        bl.setPower(Math.signum(blT) * p);
        br.setPower(Math.signum(brT) * p);
    }

    private boolean encoderMoveDone() {
        int tol = 30;
        return Math.abs(flTarget - fl.getCurrentPosition()) < tol &&
               Math.abs(frTarget - fr.getCurrentPosition()) < tol &&
               Math.abs(blTarget - bl.getCurrentPosition()) < tol &&
               Math.abs(brTarget - br.getCurrentPosition()) < tol;
    }

    private void driveFast(double inches, double power) {
        if (!moveStarted) {
            int t = (int)(inches * TICKS_PER_INCH);
            startMove(t, t, t, t, power);
            moveStarted = true;
        }
    }

    private void strafeFast(double inches, double power) {
        if (!moveStarted) {
            int t = (int)(inches * TICKS_PER_INCH * STRAFE_MULTIPLIER);
            startMove(t, -t, -t, t, power);
            moveStarted = true;
        }
    }

    private void stopDrive() {
        fl.setPower(0); fr.setPower(0); bl.setPower(0); br.setPower(0);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // -------------------- MECHANISMS --------------------
    private void intakeOn(double p) { intake.setPower(p); }
    private void intakeOff() { intake.setPower(0); }
    private void beltOn(double p) { belt.setPower(p); }
    private void beltOff() { belt.setPower(0); }
    private void shooterOn(double p) { shooterLeft.setPower(p); shooterRight.setPower(p); }
    private void shooterOff() { shooterLeft.setPower(0); shooterRight.setPower(0); }

    private void runKicker() {
        double t = stateTimer.milliseconds();
        if (t < 600) kicker.setPosition(KICKER_KICK);
        else kicker.setPosition(KICKER_REST);
    }

    // -------------------- HARDWARE INIT --------------------
    private void initHardware(HardwareMap hw) {

        fl = hw.get(DcMotorEx.class, "leftfront");
        fr = hw.get(DcMotorEx.class, "rightfront");
        bl = hw.get(DcMotorEx.class, "leftrear");
        br = hw.get(DcMotorEx.class, "rightrear");

        intake = hw.get(DcMotorEx.class, "frontintake");
        belt = hw.get(DcMotorEx.class, "belt");
        shooterLeft = hw.get(DcMotorEx.class, "leftshooter");
        shooterRight = hw.get(DcMotorEx.class, "rightshooter");

        kicker = hw.get(Servo.class, "ballkicker");

        fr.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        shooterLeft.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
}
