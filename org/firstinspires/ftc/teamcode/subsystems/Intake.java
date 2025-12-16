package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.*;

public class Intake {

    private DcMotorEx intake, belt;

    public Intake(HardwareMap hw) {
        intake = hw.get(DcMotorEx.class, "intake");
        belt = hw.get(DcMotorEx.class, "belt");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        belt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void intakeOn(double power) {
        intake.setPower(power);
    }

    public void intakeReverse(double power) {
        intake.setPower(-power);
    }

    public void intakeOff() {
        intake.setPower(0);
    }

    public void beltOn(double power) {
        belt.setPower(power);
    }

    public void beltOff() {
        belt.setPower(0);
    }
}
