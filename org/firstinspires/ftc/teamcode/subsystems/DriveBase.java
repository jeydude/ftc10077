package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.*;

public class DriveBase {

    private DcMotorEx fl, fr, bl, br;
    private LinearOpMode opMode;

    public DriveBase(LinearOpMode opMode, HardwareMap hw) {
        this.opMode = opMode;

        fl = hw.get(DcMotorEx.class, "frontLeft");
        fr = hw.get(DcMotorEx.class, "frontRight");
        bl = hw.get(DcMotorEx.class, "backLeft");
        br = hw.get(DcMotorEx.class, "backRight");

        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void drive(double inches, double power) {
        int ticks = (int) (inches * TICKS_PER_INCH);
        setTargets(ticks, ticks, ticks, ticks);
        runToPosition(power);
        // runToPositionWithBrake(power);
    }

    public void strafe(double inches, double power) {
        int ticks = (int) (inches * TICKS_PER_INCH * STRAFE_MULTIPLIER);
        setTargets(ticks, -ticks, -ticks, ticks);
        runToPosition(power);
        // runToPositionWithBrake(power);
    }

    public void turn(double degrees, double power) {
        int ticks = (int) (degrees * TICKS_PER_DEGREE);
        setTargets(ticks, -ticks, ticks, -ticks);
        runToPosition(power);
        // runToPositionWithBrake(power);
    }

    private void setTargets(int flT, int frT, int blT, int brT) {
        fl.setTargetPosition(fl.getCurrentPosition() + flT);
        fr.setTargetPosition(fr.getCurrentPosition() + frT);
        bl.setTargetPosition(bl.getCurrentPosition() + blT);
        br.setTargetPosition(br.getCurrentPosition() + brT);
    }

    private void runToPosition(double power) {
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(power);
        }

        while (opMode.opModeIsActive() &&
                (fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy())) {
            opMode.idle();
        }

        stop();
        resetEncoders();
    }

        private void runToPositionWithBrake(double power) {

        // Set RUN_TO_POSITION mode
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(power);
        }

        // Wait until all motors reach target
        while (opModeIsActive() && 
            (fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy())) {

            // Optional: telemetry for debugging
            telemetry.addData("FL", fl.getCurrentPosition());
            telemetry.addData("FR", fr.getCurrentPosition());
            telemetry.addData("BL", bl.getCurrentPosition());
            telemetry.addData("BR", br.getCurrentPosition());
            telemetry.update();
        }

        // Braking phase: reduce power gradually to zero
        double currentPower = power;
        int steps = 5;
        for (int i = steps; i >= 0; i--) {
            double p = currentPower * i / steps;
            fl.setPower(p);
            fr.setPower(p);
            bl.setPower(p);
            br.setPower(p);
            sleep(20); // short step
        }

        // Ensure motors are fully stopped
        stop();

        // Reset encoders for next move
        resetEncoders();
    }

    private void resetEncoders() {
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void stop() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
}
