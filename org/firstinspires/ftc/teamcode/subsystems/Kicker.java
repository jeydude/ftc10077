package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.*;

public class Kicker {

    private Servo kicker;
    private LinearOpMode opMode;

    public Kicker(LinearOpMode opMode, Servo kicker) {
        this.opMode = opMode;
        this.kicker = kicker;
        kicker.setPosition(KICKER_REST);
    }

    public void kick() {
        kicker.setPosition(KICKER_KICK);
        opMode.sleep(180);
        kicker.setPosition(KICKER_REST);
        opMode.sleep(150);
    }
}
