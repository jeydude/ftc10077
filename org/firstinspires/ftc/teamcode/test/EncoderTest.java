package org.firstinspires.ftc.teamcode.test;
// Hardware Imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo; 
import com.qualcomm.robotcore.hardware.HardwareMap;

// Measurement and Math Imports
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="EncoderTest")
public class EncoderTest extends LinearOpMode {

    // Define States
    enum State {
        DRIVE_TO_SHOOT,
        STRAFE_TO_SAMPLE,
        TURN_TO_BASKET,
        IDLE
    }

    State currentState = State.DRIVE_TO_SHOOT;
    
    // Hardware
    private DcMotor fl, fr, bl, br, intake, belt, shooterLeft, shooterRight;
    // Analog servo kicker
    private Servo kicker, topKicker;
    private static final double TICKS_PER_INCH = 39.35; 
    private static final double TICKS_PER_DEGREE = 9.5;
    @Override
    public void runOpMode() {
        // --- Initialization ---
        initializeHardware(hardwareMap);
        
        waitForStart();

        while (opModeIsActive()) {
            switch (currentState) {
                case DRIVE_TO_SHOOT:
                    // Drive 24 inches forward
                    if (driveInches(49, 0.8, 5.0)) {
                        currentState = State.STRAFE_TO_SAMPLE;
                    }
                    break;

                case STRAFE_TO_SAMPLE:
                    // Strafe 12 inches right
                    if (strafeInches(12, 0.8, 3.0)) {
                        currentState = State.TURN_TO_BASKET;
                    }
                    break;

                case TURN_TO_BASKET:
                    // Turn 90 degrees using encoders
                    if (encoderTurn(90, 0.8, 3.0)) {
                        currentState = State.IDLE;
                    }
                    break;

                case IDLE:
                    stopMotors();
                    break;
            }
        }
    }

    // --- Movement Methods ---

    public boolean driveInches(double inches, double speed, double timeout) {
        if (!isMoving) { 
            int target = (int)(inches * TICKS_PER_INCH);
            setTargets(target, target, target, target);
            startMove();
        }
        return monitorMotors(speed, timeout);
    }

    public boolean strafeInches(double inches, double speed, double timeout) {
        if (!isMoving) {
            int target = (int)(inches * TICKS_PER_INCH * 1.1);
            setTargets(target, -target, -target, target);
            startMove();
        }
        return monitorMotors(speed, timeout);
    }

    /**
     * Turn Left or Right using Encoders
     * Positive degrees = Right (clockwise), Negative = Left (counter-clockwise)
     */
    public boolean encoderTurn(double degrees, double speed, double timeout) {
        if (!isMoving) {
            int target = (int)(degrees * TICKS_PER_DEGREE);
            // For a right turn: Left wheels go forward (+), Right wheels go backward (-)
            setTargets(target, -target, target, -target);
            startMove();
        }
        return monitorMotors(speed, timeout);
    }

    // --- Helper Methods ---

    private void startMove() {
        moveTimer.reset();
        isMoving = true;
    }

    private void setTargets(int flT, int frT, int blT, int brT) {
        fl.setTargetPosition(fl.getCurrentPosition() + flT);
        fr.setTargetPosition(fr.getCurrentPosition() + frT);
        bl.setTargetPosition(bl.getCurrentPosition() + blT);
        br.setTargetPosition(br.getCurrentPosition() + brT);
        
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private boolean monitorMotors(double speed, double timeout) {
        fl.setPower(speed);
        fr.setPower(speed);
        bl.setPower(speed);
        br.setPower(speed);

        // Logic finishes if motors reach target OR if time runs out
        boolean reachedTarget = !fl.isBusy() && !fr.isBusy();
        boolean timedOut = moveTimer.seconds() > timeout;

        if (reachedTarget || timedOut) {
            stopMotors();
            fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
            isMoving = false; // Reset for next state
            return true;
        }
        return false;
    }

    private void stopMotors() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
    
   // -------------------- INITIALIZATION --------------------
    /*
     * Maps motors, sets directions, enables braking,
     * and resets encoders.
     */
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