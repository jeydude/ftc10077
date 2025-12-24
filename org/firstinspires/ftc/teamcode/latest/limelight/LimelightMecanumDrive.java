package org.firstinspires.ftc.teamcode.latest.limelight;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

public class LimelightMecanumDrive {

    // Motors
    private DcMotorEx fl, fr, bl, br;

    // Tunable gains
    public double kP_drive = 0.8;
    public double kP_turn  = 0.01;

    // Tolerances
    public double positionTolerance = 0.05; // meters
    public double headingTolerance  = 2.0;  // degrees

    public LimelightMecanumDrive(
            DcMotorEx frontLeft,
            DcMotorEx frontRight,
            DcMotorEx backLeft,
            DcMotorEx backRight
    ) {
        fl = frontLeft;
        fr = frontRight;
        bl = backLeft;
        br = backRight;

        // Motor directions (adjust if needed)
        fl.setDirection(DcMotorEx.Direction.REVERSE);
        bl.setDirection(DcMotorEx.Direction.REVERSE);

        // Encoder mode
        fl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Brake when stopped
        fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Call this every loop to drive to a field position
     *
     * @return true when target reached
     */
    public boolean goToPose(
            double robotX,
            double robotY,
            double robotHeading,
            double targetX,
            double targetY,
            double targetHeading
    ) {
        // Compute field-centric error
        double errorX = targetX - robotX;
        double errorY = targetY - robotY;
        double errorHeading = angleWrap(targetHeading - robotHeading);

        // Rotate error into robot frame
        double headingRad = Math.toRadians(robotHeading);

        double forward =
                errorX * Math.cos(headingRad) +
                errorY * Math.sin(headingRad);

        double strafe =
               -errorX * Math.sin(headingRad) +
                errorY * Math.cos(headingRad);

        // Apply proportional control
        double forwardPower = forward * kP_drive;
        double strafePower  = strafe  * kP_drive;
        double turnPower    = errorHeading * kP_turn;

        // Clamp
        forwardPower = Range.clip(forwardPower, -1, 1);
        strafePower  = Range.clip(strafePower,  -1, 1);
        turnPower    = Range.clip(turnPower,    -1, 1);

        // Mecanum math
        double flp = forwardPower + strafePower + turnPower;
        double frp = forwardPower - strafePower - turnPower;
        double blp = forwardPower - strafePower + turnPower;
        double brp = forwardPower + strafePower - turnPower;

        // Normalize
        double max = Math.max(1.0,
                Math.max(Math.abs(flp),
                Math.max(Math.abs(frp),
                Math.max(Math.abs(blp), Math.abs(brp)))));

        fl.setPower(flp / max);
        fr.setPower(frp / max);
        bl.setPower(blp / max);
        br.setPower(brp / max);

        // Check completion
        boolean atPosition =
                Math.hypot(errorX, errorY) < positionTolerance &&
                Math.abs(errorHeading) < headingTolerance;

        if (atPosition) stop();

        return atPosition;
    }

    public void stop() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    private double angleWrap(double degrees) {
        while (degrees > 180) degrees -= 360;
        while (degrees < -180) degrees += 360;
        return degrees;
    }
}
