package org.firstinspires.ftc.teamcode.test;
// Hardware Imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo; 
import com.qualcomm.robotcore.hardware.HardwareMap;

// Measurement and Math Imports
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="IMUTest")
public class IMUTest extends LinearOpMode {

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
    IMU imu;
    private static final double TICKS_PER_INCH = 39.35; 
    @Override
    public void runOpMode() {
        // --- Initialization ---
        initializeHardware(hardwareMap);
        
        waitForStart();

        while (opModeIsActive()) {
            switch (currentState) {
                case DRIVE_TO_SHOOT:
                    // Drive 24 inches forward
                    if (driveInches(49, 0.8)) {
                        currentState = State.STRAFE_TO_SAMPLE;
                    }
                    break;

                case STRAFE_TO_SAMPLE:
                    // Strafe 12 inches right
                    if (strafeInches(12, 0.8)) {
                        currentState = State.TURN_TO_BASKET;
                    }
                    break;

                case TURN_TO_BASKET:
                    // Turn 90 degrees using IMU
                    if (imuTurn(90, 0.8)) {
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

    public boolean driveInches(double inches, double speed) {
        int target = (int)(inches * TICKS_PER_INCH);
        setTargets(target, target, target, target);
        return monitorMotors(speed);
    }

    public boolean strafeInches(double inches, double speed) {
        // Multiplier for mecanum scrubbing (1.1)
        int target = (int)(inches * TICKS_PER_INCH * 1.1);
        setTargets(target, -target, -target, target);
        return monitorMotors(speed);
    }

    public boolean imuTurn(double targetAngle, double speed) {
        double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = targetAngle - currentAngle;

        if (Math.abs(error) < 1.0) {
            stopMotors();
            return true;
        }

        double power = Range.clip(error * 0.015, -speed, speed);
        fl.setPower(power);
        bl.setPower(power);
        fr.setPower(-power);
        br.setPower(-power);
        
        return false;
    }

    // --- Helper Methods ---

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

    private boolean monitorMotors(double speed) {
        fl.setPower(speed);
        fr.setPower(speed);
        bl.setPower(speed);
        br.setPower(speed);

        if (!fl.isBusy()) {
            stopMotors();
            // Important: Return to encoder mode after RUN_TO_POSITION finishes
            fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        
        // Initialize IMU
        imu = hw.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        
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