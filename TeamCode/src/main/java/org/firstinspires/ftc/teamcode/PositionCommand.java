package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.drive.localizer.Localizer;
import org.firstinspires.ftc.teamcode.drive.util.Pose;

@Config
public class PositionCommand extends CommandBase {
    public static PIDFController xController = new PIDFController(RobotConstants.xP, 0.0, RobotConstants.xD, RobotConstants.xF);
    public static PIDFController yController = new PIDFController(RobotConstants.yP, 0.0, RobotConstants.yD, RobotConstants.yF);
    public static PIDFController hController = new PIDFController(RobotConstants.hP, 0.0, RobotConstants.hD, RobotConstants.hF);
    public static double max_power = 1;
    public static double max_heading = 0.5;

    Drivetrain drivetrain;
    Localizer localizer;
    Pose targetPose;
    ElapsedTime deadTimer;

    private final double ms;
    private final double delay;
    private ElapsedTime delayTimer;

    private final double v;

    public PositionCommand(Drivetrain drivetrain, Localizer localizer, Pose targetPose, double delay, double dead, double voltage) {
        this.drivetrain = drivetrain;
        this.localizer = localizer;
        this.targetPose = targetPose;
        this.ms = dead;
        this.delay = delay;
        this.v = voltage;
    }

    @Override
    public void execute() {
        if (deadTimer == null) {
            deadTimer = new ElapsedTime();
        }

        Pose currentPose = targetPose.subtract(localizer.poseRobot());
        Pose powers = goToPosition(localizer.poseRobot(), targetPose);
        drivetrain.set(powers, max_power);
        Globals.currentPose = powers; // TODO: should be current pose?
    }

    @Override
    public boolean isFinished() {
        Pose error = targetPose.subtract(localizer.poseRobot());
        Globals.error = error;
        Globals.targetPose = targetPose;

        boolean reached = ((Math.hypot(error.x, error.y) < RobotConstants.ALLOWED_TRANSLATIONAL_ERROR) && (Math.abs(error.heading) < RobotConstants.ALLOWED_HEADING_ERROR));
        Globals.reached = reached;

        if (reached && delayTimer == null) {
            delayTimer = new ElapsedTime();
        }
        if (!reached && delayTimer != null) {
            delayTimer.reset();
        }

        boolean delayed = delayTimer != null && delayTimer.milliseconds() > delay;
        return (deadTimer.milliseconds() > ms) || delayed;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.set(new Pose(), max_power);
    }

    private static Pose relDistanceToTarget(Pose robot, Pose target) {
        return target.subtract(robot);
    }

    public Pose goToPosition(Pose robotPose, Pose targetPose) {
        Pose deltaPose = relDistanceToTarget(robotPose, targetPose);
        Pose powers = new Pose(
                xController.calculate(0, deltaPose.x),
                yController.calculate(0, deltaPose.y),
                hController.calculate(0, deltaPose.heading)
        );

        double x_rotated = powers.x * Math.cos(robotPose.heading) - powers.y * Math.sin(robotPose.heading);
        double y_rotated = powers.x * Math.sin(robotPose.heading) + powers.y * Math.cos(robotPose.heading);

        double x_power = -x_rotated < -max_power ? -max_power :
                Math.min(-x_rotated, max_power);
        double y_power = -y_rotated < -max_power ? -max_power :
                Math.min(-y_rotated, max_power);
        double heading_power = powers.heading;

        heading_power = Math.max(Math.min(max_heading, heading_power), -max_heading);

        return new Pose(-y_power / v * 12.5, x_power / v * 12.5, -heading_power / v * 12.5);
    }
}
