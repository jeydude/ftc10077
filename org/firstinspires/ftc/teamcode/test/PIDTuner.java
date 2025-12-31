package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="PID_Tuner_With_Voltage", group="Tuning")
public class PIDTuner extends LinearOpMode {

    private DcMotorEx fl, fr, bl, br;
    private VoltageSensor batterySensor;
    
    // Starting values
    double Kp = 0.04, Ki = 0.0, Kd = 0.0;
    double increment = 0.001;
    
    double integralSum = 0, lastError = 0;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Init hardware
        fl = hardwareMap.get(DcMotorEx.class, "leftfront");
        fr = hardwareMap.get(DcMotorEx.class, "rightfront");
        bl = hardwareMap.get(DcMotorEx.class, "leftrear");
        br = hardwareMap.get(DcMotorEx.class, "rightrear");
        
        // Get the voltage sensor from the hardware map
        batterySensor = hardwareMap.voltageSensor.iterator().next();
        
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
        
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            // Live Tuning Logic
            if (gamepad1.dpad_up) Kp += increment;
            if (gamepad1.dpad_down) Kp -= increment;
            if (gamepad1.dpad_right) Kd += increment;
            if (gamepad1.dpad_left) Kd -= increment;
            if (gamepad1.x) Ki -= 0.00001;
            if (gamepad1.b) Ki += 0.00001;

            // Run Test
            if (gamepad1.a) {
                runTest(48); 
                sleep(500);
                runTest(-48);
            }

            // Display live diagnostics
            telemetry.addLine("--- BATTERY STATUS ---");
            double currentVoltage = batterySensor.getVoltage();
            telemetry.addData("Current Voltage", "%.2fV", currentVoltage);
            
            // Visual warning for low battery
            if (currentVoltage < 12.0) {
                telemetry.addLine("!!! WARNING: BATTERY LOW - PID MAY BE SLUGGISH !!!");
            }

            telemetry.addLine("\n--- PID VALUES ---");
            telemetry.addData("Kp (Proportional)", "%.4f", Kp);
            telemetry.addData("Ki (Integral)", "%.6f", Ki);
            telemetry.addData("Kd (Derivative)", "%.4f", Kd);
            
            telemetry.addLine("\n--- CONTROLS ---");
            telemetry.addLine("DPAD Up/Down: Kp | DPAD Left/Right: Kd");
            telemetry.addLine("X/B Buttons: Ki | PRESS A: Run 48\" Test");
            telemetry.update();
            
            sleep(100); 
        }
    }

    public void runTest(double targetInches) {
        double targetTicks = targetInches * 39.35;
        resetEncoders();
        
        timer.reset();
        integralSum = 0;
        lastError = 0;

        while (opModeIsActive() && timer.seconds() < 3.5) {
            double currentPos = fl.getCurrentPosition();
            double error = targetTicks - currentPos;

            // Basic PID calculation
            integralSum += error * timer.seconds();
            double derivative = (error - lastError) / timer.seconds();
            lastError = error;

            double output = (error * Kp) + (integralSum * Ki) + (derivative * Kd);
            
            // Apply power
            double power = Range.clip(output, -0.7, 0.7);
            setAllPower(power);

            // Exit if error is minimal
            if (Math.abs(error) < 12) break;
        }
        setAllPower(0);
    }

    private void resetEncoders() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setAllPower(double p) {
        fl.setPower(p); fr.setPower(p); bl.setPower(p); br.setPower(p);
    }
}