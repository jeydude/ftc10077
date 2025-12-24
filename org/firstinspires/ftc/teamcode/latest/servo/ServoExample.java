package org.firstinspires.ftc.teamcode.latest.servo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ServoExample", group="Tutorial")
public class ServoExample extends LinearOpMode {

    Servo armServo;

    @Override
    public void runOpMode() {
        // Map the servo from configuration
        armServo = hardwareMap.get(Servo.class, "armServo");

        // Set starting position
        // armServo.setPosition(0.0);

        waitForStart();

        while (opModeIsActive()) {
            // Example: Move servo to different positions
            if (gamepad1.a) {
                armServo.setPosition(0.0); // Move to 0% position
            } else if (gamepad1.b) {
                armServo.setPosition(1.0); // Move to 100% position
            } else if (gamepad1.y) {
                armServo.setPosition(0.5); // Move to 50% (middle)
            } else if (gamepad1.x) {
                moveServo(armServo, armServo.getPosition(), 1.0, 2.0, 20); // Smooth move to 100% over 2 seconds
            }
            
            telemetry.addData("Servo Position", armServo.getPosition());
            telemetry.update();
        }
    }
    
    /**
     * Moves a servo from start to end position smoothly over the given time.
     * @param servo The servo object
     * @param startPos Starting position (0.0 - 1.0)
     * @param endPos Ending position (0.0 - 1.0)
     * @param timeSeconds Total time to move in seconds
     * @param steps Number of steps for smooth motion (more steps = smoother)
     */
    public static void moveServo(Servo servo, double startPos, double endPos, double timeSeconds, int steps) {
        double stepSize = (endPos - startPos) / steps;
        long delayMillis = (long)((timeSeconds * 1000) / steps);
        double position = startPos;

        for (int i = 0; i <= steps; i++) {
            servo.setPosition(position);
            position += stepSize;
            try {
                Thread.sleep(delayMillis);
            } catch (InterruptedException e) {
                // Handle interruption (if OpMode is stopped)
                break;
            }
        }

        // Ensure servo is exactly at the end position
        servo.setPosition(endPos);
    }
}