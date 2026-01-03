package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo; 
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="VelocityTest")
public class VelocityTest extends LinearOpMode {

    enum State { DRIVE_TO_SHOOT, STRAFE_TO_SAMPLE, TURN_TO_BASKET, IDLE }
    State currentState = State.DRIVE_TO_SHOOT;
    
    private DcMotorEx fl, fr, bl, br, intake, belt, shooterLeft, shooterRight;
    private Servo kicker, topKicker;

    // goBILDA 312 RPM constants
    private static final double TICKS_PER_INCH = 39.35; 
    private static final double TICKS_PER_DEGREE = 9.5;
    // 312 RPM Max Velocity is ~2800. We use 2000 to give the PID "headroom"
    private static final double MAX_VELOCITY_TICKS = 2000; 

    private ElapsedTime moveTimer = new ElapsedTime();
    private boolean isMoving = false;
    private int[] targets = new int[4];

    @Override
    public void runOpMode() {
        initializeHardware(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            switch (currentState) {
                case DRIVE_TO_SHOOT:
                    if (driveInches(49, 0.8, 5.0)) currentState = State.STRAFE_TO_SAMPLE;
                    break;

                case STRAFE_TO_SAMPLE:
                    if (strafeInches(12, 0.8, 3.0)) currentState = State.TURN_TO_BASKET;
                    break;

                case TURN_TO_BASKET:
                    if (encoderTurn(90, 0.8, 3.0)) currentState = State.IDLE;
                    break;

                case IDLE:
                    stopMotors();
                    break;
            }
        }
    }

    public boolean driveInches(double inches, double speed, double timeout) {
        if (!isMoving) { 
            int t = (int)(inches * TICKS_PER_INCH);
            setPIDTargets(t, t, t, t);
        }
        return monitorPIDMove(speed, timeout);
    }

    public boolean strafeInches(double inches, double speed, double timeout) {
        if (!isMoving) {
            int t = (int)(inches * TICKS_PER_INCH * 1.1); // 1.1 accounts for strafe slip
            setPIDTargets(t, -t, -t, t);
        }
        return monitorPIDMove(speed, timeout);
    }

    public boolean encoderTurn(double degrees, double speed, double timeout) {
        if (!isMoving) {
            int t = (int)(degrees * TICKS_PER_DEGREE);
            setPIDTargets(t, -t, t, -t);
        }
        return monitorPIDMove(speed, timeout);
    }

    private void setPIDTargets(int flT, int frT, int blT, int brT) {
        targets[0] = fl.getCurrentPosition() + flT;
        targets[1] = fr.getCurrentPosition() + frT;
        targets[2] = bl.getCurrentPosition() + blT;
        targets[3] = br.getCurrentPosition() + brT;
        
        moveTimer.reset();
        isMoving = true;
    }

    private boolean monitorPIDMove(double speed, double timeout) {
        // Calculate distance remaining for one motor (fl)
        int remaining = Math.abs(targets[0] - fl.getCurrentPosition());
        
        // If we are within 15 ticks or timed out, stop
        if (remaining < 15 || moveTimer.seconds() > timeout) {
            stopMotors();
            isMoving = false;
            return true;
        }

        // Apply constant velocity using PIDF
        // We use Math.signum to ensure the velocity goes the right direction for each wheel
        fl.setVelocity(speed * MAX_VELOCITY_TICKS * Math.signum(targets[0] - fl.getCurrentPosition()));
        fr.setVelocity(speed * MAX_VELOCITY_TICKS * Math.signum(targets[1] - fr.getCurrentPosition()));
        bl.setVelocity(speed * MAX_VELOCITY_TICKS * Math.signum(targets[2] - bl.getCurrentPosition()));
        br.setVelocity(speed * MAX_VELOCITY_TICKS * Math.signum(targets[3] - br.getCurrentPosition()));

        return false;
    }

    private void stopMotors() {
        fl.setVelocity(0);
        fr.setVelocity(0);
        bl.setVelocity(0);
        br.setVelocity(0);
    }
    
    private void initializeHardware(HardwareMap hw) {

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

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        belt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // // Shooter motors should coast for smoother RPM
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Brake so ball doesn't roll backward when stopped
        belt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

  
    }
}