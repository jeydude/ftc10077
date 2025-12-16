package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.*;
import org.firstinspires.ftc.teamcode.subsystems.*;

@Autonomous(name = "AutonomousTest", group = "Drive")
public class AutonomousTest extends LinearOpMode {

    private DriveBase drive;
    private Intake intake;
    private Shooter shooter;
    private Kicker kicker;

    @Override
    public void runOpMode() {

        drive = new DriveBase(this, hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        kicker = new Kicker(this, hardwareMap.get(Servo.class, "kicker"));

        waitForStart();

        if (!opModeIsActive()) return;

        telemetry.addLine("Autonomous Ready");
        telemetry.update();

        // Wait for the play button to be pressed
        waitForStart();

        if (opModeIsActive()) {

            // ----- FIRST 3 BALLS -----
            // Drive backward 24 inches
            drive.drive(-24, 0.6);

            // Spin up shooter ONCE
            double shooterPower = shooter.calculatePower();
            shooter.on(shooterPower);
            sleep(1000);

            // -------- BALL 1 --------
            kicker.kick();
            sleep(120);

            intake.beltOn(0.5);
            sleep(250);
            intake.beltOff();
            sleep(150);

            // -------- BALL 2 --------
            kicker.kick();
            sleep(120);

            intake.intakeOn(0.8);
            intake.beltOn(0.5);
            sleep(250);
            intake.beltOff();
            intake.intakeOff();
            sleep(150);
            

            // -------- BALL 3 --------
            kicker.kick();
            sleep(200);

            // Shut down shooter
            shooter.off();
            // ----- COLLECT NEXT 3 BALLS -----
            // Move to collect another 3 balls
            // Strafe right 18 inches
            drive.strafe(18, 0.5);
            // Turn 90 degrees clockwise
            drive.turn(90, 0.45);

            // Drive backward 12 inches
            drive.drive(-12, 0.4);

            // Start intake before moving
            intake.intakeOn(1.0);

            // Drive forward to collect the ball
            drive.drive(30, 0.4);

            // Give intake time to fully pull in the ball
            sleep(300);

            // Stop the intake after collecting the ball
            intake.intakeOff();

            // Optional: clear last ball
            intake.intakeReverse(0.4);
            sleep(100);
            intake.intakeOff();

            // ----- RETURN TO SHOOTING POSITION -----
            drive.drive(-30, 0.4);  // Adjust distance based on field
            drive.turn(-90, 0.45);  // Turn back to face goal
            drive.strafe(-18, 0.5); // Strafe back to original lane
            drive.drive(-12, 0.4);  // Final approach to shooting position
            // ----- SHOOT NEXT 3 BALLS -----
            // Spin up shooter
            shooterPower = shooter.calculatePower();
            shooter.on(shooterPower);
            sleep(1500);
            // -------- BALL 4 --------
            kicker.kick();
            sleep(120);

            intake.beltOn(0.5);
            sleep(250);
            intake.beltOff();
            sleep(150);
            // -------- BALL 5 --------
            kicker.kick();
            sleep(120);

            intake.intakeOn(0.8);
            intake.beltOn(0.5);
            sleep(250);
            intake.beltOff();
            intake.intakeOff();
            sleep(150);
            // -------- BALL 6 --------
            kicker.kick();
            sleep(200);

            // Shut down shooter
            shooter.off();

        }
    }
}
