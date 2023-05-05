package org.firstinspires.ftc.teamcode.drive.localizer;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.util.Pose;

import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;

@Config
public class TwoWheelLocalizer extends TwoTrackingWheelLocalizer implements Localizer {

    private final DoubleSupplier horizontalPosition, lateralPosition, imuAngle;
    private final DoubleSupplier horizontalVelocity, lateralVelocity;

    public TwoWheelLocalizer(RobotHardware robot) {
        super(Arrays.asList(
                new Pose2d(RobotConstants.PARALLEL_X, RobotConstants.PARALLEL_Y, 0),
                new Pose2d(RobotConstants.PERPENDICULAR_X, RobotConstants.PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.horizontalPosition = () -> robot.parallelPod.getPosition();
        this.lateralPosition = () -> robot.leftPerpindicularPod.getPosition();

        // TODO: check if raw or corrected better
        this.horizontalVelocity = () -> robot.parallelPod.getRawVelocity();
        this.lateralVelocity = () -> robot.leftPerpindicularPod.getRawVelocity();

        this.imuAngle = robot::getAngle;
    }

    public static double encoderTicksToInches(double ticks) {
        return RobotConstants.WHEEL_RADIUS_DEAD_WHEELS * 2 * Math.PI * RobotConstants.GEAR_RATIO_DEAD_WHEELS * ticks / RobotConstants.TICKS_PER_REV_REV_BORE_ENCODER;
    }

    @Override
    public double getHeading() {
        return imuAngle.getAsDouble();
    }

    @Override
    public Double getHeadingVelocity() {
        return 0.0;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(horizontalPosition.getAsDouble()),
                encoderTicksToInches(lateralPosition.getAsDouble())
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(horizontalVelocity.getAsDouble(), lateralVelocity.getAsDouble());
    }

    @Override
    public Pose poseRobot() {
        Pose2d pose = getPoseEstimate();
        return new Pose(pose.getX(), -pose.getY(), pose.getHeading());
    }

    @Override
    public Pose poseVelocity() {
        Pose2d pose = getPoseVelocity();
        return new Pose(pose.getX(), pose.getY(), pose.getHeading());
    }

    @Override
    public void setPoseRobot(Pose pose) {
        setPoseEstimate(new Pose2d(pose.x, pose.y, pose.heading));
    }

    @Override
    public void setPoseVelocity(Pose pose) {
        setPoseVelocity(new Pose2d(pose.x, pose.y, pose.heading));
    }

    @Override
    public void update() {
        super.update();
    }
}