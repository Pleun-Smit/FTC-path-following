package org.firstinspires.ftc.teamcode;

public class Constants {
    // ALLE ZUT STAAT HIER IN CENTIMETERS

    public static final double WHEEL_RADIUS = 1.9;
    public static final double TICKS_PER_REV = 1440;
    public static final double CENTIMETERS_PER_REV = 2 * Math.PI * WHEEL_RADIUS;
    public static final double TICKS_TO_CENTIMETERS = 1 / TICKS_PER_REV * CENTIMETERS_PER_REV;
    public static final double TRACK_WIDTH = 20.9;
    public static final double FORWARD_OFFSET = -3.6;
    public static final double TURNING_KP = 1 / Math.PI;
}
