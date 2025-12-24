package org.firstinspires.ftc.teamcode.latest.limelight;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.limelightvision.limelightvisionlib.Limelight3A;
import com.limelightvision.limelightvisionlib.LLResult;
import com.limelightvision.limelightvisionlib.Pose3D;
import org.firstinspires.ftc.teamcode.latest.limelight.LimelightMecanumDrive;

@Autonomous(name="Limelight Auto Test")
public class LimelightAutoTest extends LinearOpMode {

    Limelight3A limelight;
    LimelightMecanumDrive drive;

    @Override
    public void runOpMode() {

        // --------------------
        // Drivetrain hardware
        // --------------------
        DcMotorEx fl = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx fr = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx bl = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx br = hardwareMap.get(DcMotorEx.class, "backRight");

        drive = new LimelightMecanumDrive(fl, fr, bl, br);

        // --------------------
        // Limelight hardware
        // --------------------
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Turn on LEDs
        limelight.setLEDMode(Limelight3A.LEDMode.ON);

        // Use vision processing camera mode
        limelight.setCameraMode(Limelight3A.CameraMode.VISION);

        // Select pipeline (APRILTAG pipeline index!)
        limelight.setPipeline(0);

        telemetry.addLine("Limelight Initialized");
        telemetry.update();

        waitForStart();

        // --------------------
        // Main loop
        // --------------------
        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                Pose3D botPose = result.getBotpose();

                if (botPose != null) {
                    boolean arrived = drive.goToPose(
                            botPose.getX(),       // meters
                            botPose.getY(),
                            botPose.getYaw(),     // degrees
                            1.5,                  // target X (meters)
                            0.75,                 // target Y (meters)
                            90                    // target heading
                    );

                    telemetry.addData("Arrived", arrived);
                }
            } else {
                telemetry.addLine("No Limelight Target");
                drive.stop();
            }

            telemetry.update();
        }
    }
}
