/*
Copyright 2025 FIRST Tech Challenge Team 10077

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains a minimal example of a Linear "OpMode". An OpMode is a 'program' that runs
 * in either the autonomous or the TeleOp period of an FTC match. The names of OpModes appear on
 * the menu of the FTC Driver Station. When an selection is made from the menu, the corresponding
 * OpMode class is instantiated on the Robot Controller and executed.
 *
 * Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
 * added to the Driver Station.
 */
@Autonomous(name="AutoBLUE", group="Autonomous")
public class autoblue extends LinearOpMode {

    //Declare hardware components
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor intake = null;
    private DcMotor leftshooter = null;
    private DcMotor rightshooter = null;
    private DcMotor belt = null;
    private Servo kicker = null;
    private double shooter_speed = 1.0;
    

    // Define constants for wheel circumference, encoder ticks per revolution, etc.
    static final double     COUNTS_PER_MOTOR_REV    = 537.6;    // GoBilda 5202-series Yellow Jacket motor
    static final double     DRIVE_GEAR_REDUCTION   = 1.0;     // This is the gear reduction on the drive motor. Set to 1.0 for direct drive.
    static final double     WHEEL_DIAMETER_INCHES  = 4.0;     // For GoBilda Mecanum wheels
    static final double     COUNTS_PER_INCH        = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED            = 0.7;

    private ElapsedTime     runtime = new ElapsedTime();
     /**
     * Returns the minimum voltage reported by any voltage sensor on the hardware map.
     * @return The lowest voltage reported.
     */
    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        // Loop through all voltage sensors on the hardware map
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
    
    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "leftfront");
        frontRight = hardwareMap.get(DcMotor.class, "rightfront");
        backLeft = hardwareMap.get(DcMotor.class, "leftrear");
        backRight = hardwareMap.get(DcMotor.class, "rightrear");
        intake = hardwareMap.get(DcMotor.class, "frontintake");
        leftshooter = hardwareMap.get(DcMotor.class, "leftshooter");
        rightshooter = hardwareMap.get(DcMotor.class, "rightshooter");
        belt = hardwareMap.get(DcMotor.class, "belt");
        // Initialize the servo from the hardware map
        kicker = hardwareMap.get(Servo.class, "ballkicker");
        

        // Set motor directions (adjust as needed for your robot)
        frontLeft.setDirection(DcMotor.Direction.FORWARD);//reverse
        backLeft.setDirection(DcMotor.Direction.REVERSE);//forward
        frontRight.setDirection(DcMotor.Direction.REVERSE);//forward
        backRight.setDirection(DcMotor.Direction.FORWARD);//reverse
        intake.setDirection(DcMotor.Direction.FORWARD);
        leftshooter.setDirection(DcMotor.Direction.REVERSE);
        rightshooter.setDirection(DcMotor.Direction.FORWARD);
        
        // This is crucial for using encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // Set motor run mode
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftshooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightshooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        //To avoid random movement of chassis while stopping the robot
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftshooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightshooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          frontLeft.getCurrentPosition(),
                          frontRight.getCurrentPosition());
        telemetry.update();

        waitForStart();
        
        shooter_speed = getShooterSpeedFromVoltage(); 
        
        //start the shooter
        leftshooter.setPower(shooter_speed);
        rightshooter.setPower(shooter_speed);
        //Drives forward
        encoderDrive(DRIVE_SPEED, 40, 40, 5.0);  // Drive forward 48 inches
        // sleep(500);
        //Shoots all the balls
        telemetry.addData("shooter speed", shooter_speed);
        telemetry.addData("battery level", getBatteryVoltage());
        telemetry.update();


        shoot_all();
        leftshooter.setPower(0);
        rightshooter.setPower(0);
        
        //strafe for 25 inches
        // strafe(-25, 1.0, 1);//strafe right
        this.shootnextballs();

        telemetry.addData("shooter speed", shooter_speed);
        telemetry.addData("battery level", getBatteryVoltage());
        telemetry.addData("Path", "Complete");

        telemetry.update();
    }
    public void shoot_all(){
        shoot(1);//shoot first 1 ball
        shoot(2);
        shoot(3);
    }
        
    public void shoot(int number){
        //Stops the shooter first
        // leftshooter.setPower(1);
        // rightshooter.setPower(1);
        if (number == 2|| number == 5) {
            //move the balls using belt
            belt.setPower(1);
            sleep(500);
        } else if (number == 3|| number == 6) {
            //Bring the ball up using intake before shooting 3rd ball
            intake.setPower(1);
            sleep(400);
            intake.setPower(0);
            //move the balls up using the belt
            belt.setPower(1);
            sleep(1200);
        }
        belt.setPower(0);
        // //Starts the shooter
        // if (number == 2){
        //     leftshooter.setPower(shooter_speed-0.0);
        //     rightshooter.setPower(shooter_speed-0.0);
        // } else{
        //     leftshooter.setPower(shooter_speed);
        //     rightshooter.setPower(shooter_speed);
        // }
        // sleep(1000);
        //Brings the kicker up to shoot
        kicker.setPosition(-0.8); 
        sleep(1200);
        //Brings the kicker back to its original position
        kicker.setPosition(0.5);
        sleep(500);
        //Stops the shooter
        // leftshooter.setPower(0);
        // rightshooter.setPower(0);
    }
    
    public double getShooterSpeedFromVoltage() {
        double v = getBatteryVoltage();

        if (v >= 14.0) return 0.83;
        else if (v >= 13.9) return 0.85;
        else if (v >= 13.5) return 0.865;
        else if (v >= 13.3) return 0.87;
        else if (v >= 13.2) return 0.88;
        else if (v >= 13.1) return 0.89;
        else if (v >= 13.0) return 0.90;
        else if (v >= 12.9) return 0.91;
        else if (v >= 12.7) return 0.92;
        else if (v >= 12.5) return 0.94;
        else if (v >= 12.0) return 0.95;
        else return 1.0;
    }
    
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        if (opModeIsActive()) {

            newLeftFrontTarget = frontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newLeftBackTarget = backLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = frontRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRightBackTarget = backRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            frontLeft.setTargetPosition(newLeftFrontTarget);
            backLeft.setTargetPosition(newLeftBackTarget);
            frontRight.setTargetPosition(newRightFrontTarget);
            backRight.setTargetPosition(newRightBackTarget);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            runtime.reset();
            frontLeft.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (frontLeft.isBusy() && frontRight.isBusy())) {
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            frontLeft.getCurrentPosition(),
                                            frontRight.getCurrentPosition());
                
                telemetry.update();
            }

            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
            
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void strafe(double distanceInches, double power, int direction) {
        int targetCounts = (int) (distanceInches * COUNTS_PER_INCH);

        // Calculate target positions for each wheel for strafing right
        int flTarget = frontLeft.getCurrentPosition() + targetCounts;
        int frTarget = frontRight.getCurrentPosition() - targetCounts;
        int blTarget = backLeft.getCurrentPosition() - targetCounts;
        int brTarget = backRight.getCurrentPosition() + targetCounts;
        //left strafing
        if (direction == 1) {
            // for strafing left
            flTarget = frontLeft.getCurrentPosition() - targetCounts;
            frTarget = frontRight.getCurrentPosition() + targetCounts;
            blTarget = backLeft.getCurrentPosition() + targetCounts;
            brTarget = backRight.getCurrentPosition() - targetCounts;
        }

        frontLeft.setTargetPosition(flTarget);
        frontRight.setTargetPosition(frTarget);
        backLeft.setTargetPosition(blTarget);
        backRight.setTargetPosition(brTarget);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while (opModeIsActive() &&
               (frontLeft.isBusy() || frontRight.isBusy() ||
                backLeft.isBusy() || backRight.isBusy())) {
            // Wait for motors to reach target
            telemetry.addData("Current Pos FL", frontLeft.getCurrentPosition());
            telemetry.addData("Current Pos FR", frontRight.getCurrentPosition());
            telemetry.addData("Current Pos BL", backLeft.getCurrentPosition());
            telemetry.addData("Current Pos BR", backRight.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // Reset run mode
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    // --- ADDED ENCODER TURN METHOD ---
    public void encoderTurn(double speed, double degrees, double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;
        
        // You MUST determine the correct COUNTS_PER_DEGREE for your robot. 
        // Calibrate this value on the field for accuracy.
        final double COUNTS_PER_DEGREE = 9.7; // **CALIBRATE THIS VALUE**

        int turnTicks = (int)(degrees * COUNTS_PER_DEGREE);

        if (opModeIsActive()) {
            // To turn clockwise, left motors go forward (+ticks), right motors go backward (-ticks).
            newFrontLeftTarget = frontLeft.getCurrentPosition() + turnTicks;
            newBackLeftTarget = backLeft.getCurrentPosition() + turnTicks;
            newFrontRightTarget = frontRight.getCurrentPosition() - turnTicks;
            newBackRightTarget = backRight.getCurrentPosition() - turnTicks;
            
            // Set Target Position
            frontLeft.setTargetPosition(newFrontLeftTarget);
            backLeft.setTargetPosition(newBackLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            backRight.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Start the motors
            runtime.reset();
            frontLeft.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));

            // Keep looping while motors are busy and timeout has not been reached
            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                   (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {
                // Optional: Add telemetry
                telemetry.addData("Turning to",  "Target: %7d", turnTicks);
                telemetry.addData("Motor Positions", "LF:%7d RF:%7d LB:%7d RB:%7d",
                                  frontLeft.getCurrentPosition(),
                                  frontRight.getCurrentPosition(),
                                  backLeft.getCurrentPosition(),
                                  backRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);

            // Turn off RUN_TO_POSITION mode
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    
    public void shootnextballs(){
        //after first 3 balls shooted, turn to get next 3 balls
        //Drives forward
        encoderDrive(0.7, 5, 5, 3);
        encoderTurn(DRIVE_SPEED, -130, 5.0); 
        strafe(7, 1.0, 1);//strafe left
        //Collect next three balls
         intake.setPower(1);
         belt.setPower(0.5);
         encoderDrive(0.3, 38, 38, 5);
         sleep(250);
         belt.setPower(0);
         //encoderDrive(0.5, 5, 5, 5);
         intake.setPower(0);
         
         //bring the ball little down
         bringballdown();
         
        //Go back to shoot
         encoderDrive(0.6, -30, -30, 5);
         sleep(800);
         encoderTurn(DRIVE_SPEED, 145, 5.0); 
         strafe(15, 1.0, 1);//strafe left
        //  encoderTurn(DRIVE_SPEED, 15, 5.0); 
         
         //start the shooter
        shooter_speed = getShooterSpeedFromVoltage();
        leftshooter.setPower(shooter_speed);
        rightshooter.setPower(shooter_speed);
        telemetry.addData("shooter speed", shooter_speed);
        telemetry.addData("battery level", getBatteryVoltage());
        telemetry.update();
        shoot(4);
        shoot(5);
        shoot(6);
        leftshooter.setPower(0);
        rightshooter.setPower(0);
        strafe(-25, 1.0, 1);//strafe right

         
    }
    public void bringballdown(){
         leftshooter.setPower(-0.5);
         rightshooter.setPower(-0.5);
         intake.setPower(-0.5);
         belt.setPower(-0.8);
         sleep(250);
         belt.setPower(0); 
         intake.setPower(0);
         leftshooter.setPower(0);
         rightshooter.setPower(0);         
    }
    
}



