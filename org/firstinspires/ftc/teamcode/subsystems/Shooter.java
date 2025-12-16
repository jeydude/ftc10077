package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.*;

public class Shooter {

    private DcMotorEx left, right;
    private HardwareMap hw;

    public Shooter(HardwareMap hw) {
        this.hw = hw;

        left = hw.get(DcMotorEx.class, "shooterLeft");
        right = hw.get(DcMotorEx.class, "shooterRight");

        right.setDirection(DcMotor.Direction.REVERSE);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void on(double power) {
        left.setPower(power);
        right.setPower(power);
    }

    public void off() {
        left.setPower(0);
        right.setPower(0);
    }

    public double calculatePower() {
        double voltage = 12.0;
        for (VoltageSensor v : hw.voltageSensor) {
            if (v.getVoltage() > 0) {
                voltage = v.getVoltage();
                break;
            }
        }

        double minV = 12.0, maxV = 13.8;
        double minP = 0.85, maxP = 1.0;

        double power = maxP - ((voltage - minV) / (maxV - minV)) * (maxP - minP);
        return Math.max(minP, Math.min(maxP, power));
    }
}
