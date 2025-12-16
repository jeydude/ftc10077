package org.firstinspires.ftc.teamcode.util;

public class RobotConstants {

    // Encoder & drive constants
    public static final double TICKS_PER_REV = 537.6;
    public static final double WHEEL_DIAMETER_INCHES = 4.0;
    public static final double GEAR_RATIO = 1.0;

    public static final double TICKS_PER_INCH =
            (TICKS_PER_REV * GEAR_RATIO) /
            (Math.PI * WHEEL_DIAMETER_INCHES);

    public static final double STRAFE_MULTIPLIER = 1.1;
    public static final double TICKS_PER_DEGREE = 10.8;

    // Kicker servo
    public static final double KICKER_REST = 0.5;
    public static final double KICKER_KICK = -0.8;
}
