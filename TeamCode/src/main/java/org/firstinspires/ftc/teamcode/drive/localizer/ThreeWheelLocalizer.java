package org.firstinspires.ftc.teamcode.drive.localizer;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.util.Pose;

import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class ThreeWheelLocalizer extends ThreeTrackingWheelLocalizer implements Localizer{
    private final DoubleSupplier leftPosition, rightPosition, frontPosition;
    private final DoubleSupplier leftVelocity, rightVelocity, frontVelocity;

    public ThreeWheelLocalizer(RobotHardware robot, List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels) {
        super(Arrays.asList(
                new Pose2d(0, RobotConstants.LATERAL_DISTANCE_DEAD_WHEELS / 2, 0), // left
                new Pose2d(0, -RobotConstants.LATERAL_DISTANCE_DEAD_WHEELS / 2, 0), // right
                new Pose2d(RobotConstants.FORWARD_OFFSET_DEAD_WHEELS, 0, Math.toRadians(90)) // front
        ));

        this.frontPosition = () -> robot.parallelPod.getPosition();
        this.leftPosition = () -> robot.leftPerpindicularPod.getPosition();
        this.rightPosition = () -> robot.rightPerpindicularPod.getPosition();

        // TODO: check if raw or corrected better
        this.frontVelocity = () -> robot.parallelPod.getRawVelocity();
        this.leftVelocity = () -> robot.leftPerpindicularPod.getRawVelocity();
        this.rightVelocity = () -> robot.rightPerpindicularPod.getRawVelocity();

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public static double encoderTicksToInches(double ticks) {
        return RobotConstants.WHEEL_RADIUS_DEAD_WHEELS * 2 * Math.PI * RobotConstants.GEAR_RATIO_DEAD_WHEELS * ticks / RobotConstants.TICKS_PER_REV_REV_BORE_ENCODER;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        int leftPos = (int) leftPosition.getAsDouble();
        int rightPos = (int) rightPosition.getAsDouble();
        int frontPos = (int) frontPosition.getAsDouble();

        return Arrays.asList(
                encoderTicksToInches(leftPos),
                encoderTicksToInches(rightPos),
                encoderTicksToInches(frontPos)
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        int leftVel = (int) leftVelocity.getAsDouble();
        int rightVel = (int) rightVelocity.getAsDouble();
        int frontVel = (int) frontVelocity.getAsDouble();

        return Arrays.asList(
                encoderTicksToInches(leftVel),
                encoderTicksToInches(rightVel),
                encoderTicksToInches(frontVel)
        );
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
