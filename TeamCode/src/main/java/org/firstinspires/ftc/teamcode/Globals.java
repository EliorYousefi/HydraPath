package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.drive.util.Pose;

@Config
public class Globals {

    public static Pose currentPose = new Pose(0, 0, 0);
    public static Pose error = new Pose(0, 0, 0);
    public static Pose targetPose = new Pose(0, 0, 0);
    public static boolean reached = false;

    public static boolean USING_IMU = true;

    public static int wait1 = 100;
    public static int wait2 = 300;
    public static int wait3 = 50;
    public static int wait4 = 50;
    public static int wait5 = 200;
    public static int wait6 = 500;
}
