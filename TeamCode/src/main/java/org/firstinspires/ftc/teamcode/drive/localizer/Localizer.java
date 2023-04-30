package org.firstinspires.ftc.teamcode.drive.localizer;

import org.firstinspires.ftc.teamcode.drive.util.Pose;

public interface Localizer {

    void update();

    /**
     * Current robot pose estimate.
     */
    Pose poseRobot();

    Pose poseVelocity();

    void setPoseRobot(Pose pose);

    void setPoseVelocity(Pose pose);
}