package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

/**
 * Units are inches :0
 */

@Config
public class RobotConstants {
    public static double kV = 0;
    public static double kA = 0;
    public static double kStatic = 0;
    public static double trackWidth = 0;
    public static double trackLength = 0;
    public static double lateralMultiplayer = 0;

    public static double TICKS_PER_REV_REV_BORE_ENCODER = 8192; // TODO: tune
    public static double WHEEL_RADIUS_DEAD_WHEELS = .98; // in TODO: tune
    public static double GEAR_RATIO_DEAD_WHEELS = 1; // output (wheel) speed / input (encoder) speed TODO: tune
    public static double LATERAL_DISTANCE_DEAD_WHEELS = 10; // in; distance between the left and right wheels TODO: tune
    public static double FORWARD_OFFSET_DEAD_WHEELS = 4; // in; offset of the lateral wheel TODO: tune


    // TODO: TUNE
    public static double PARALLEL_X = 0; // front wheel
    public static double PARALLEL_Y = 0; // front wheel

    // TODO: TUNE
    public static double PERPENDICULAR_X = 0; // left/right wheel
    public static double PERPENDICULAR_Y = 0; // left/right wheel


    public static double TICKS_PER_REV_GO_BILDA = 1; // rev bore encoder
    public static double WHEEL_RADIUS_MECHANUM = 1.88976378;
    public static double GEAR_RATIO_MECHANUM = 1;
}
