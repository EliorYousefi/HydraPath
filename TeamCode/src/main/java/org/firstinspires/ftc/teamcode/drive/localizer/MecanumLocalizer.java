package org.firstinspires.ftc.teamcode.drive.localizer;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.util.Pose;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class MecanumLocalizer extends MecanumDrive implements Localizer {
    private List<DcMotorEx> motors;

    DcMotorEx frontLeft, frontRight, backLeft, backRight;

    IMU imu;

    public MecanumLocalizer(RobotHardware robot) {
        super(RobotConstants.kV, RobotConstants.kA, RobotConstants.kStatic, RobotConstants.trackWidth,
                RobotConstants.trackLength, RobotConstants.lateralMultiplayer);

        frontLeft = robot.frontLeftMotor;
        frontRight = robot.frontRightMotor;
        backLeft = robot.backLeftMotor;
        backRight = robot.backRightMotor;

        imu = robot.imu;

        motors = Arrays.asList(frontLeft, backLeft, backRight, frontRight);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

    }


    public static double encoderTicksToInches(double ticks) {
        return RobotConstants.WHEEL_RADIUS_MECHANUM * 2 * Math.PI * RobotConstants.GEAR_RATIO_MECHANUM * ticks / RobotConstants.TICKS_PER_REV_GO_BILDA;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int position = motor.getCurrentPosition();
            wheelPositions.add(encoderTicksToInches(position));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int vel = (int) motor.getVelocity();
            wheelVelocities.add(encoderTicksToInches(vel));
        }
        return wheelVelocities;
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
        setPoseVelocity(pose);
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return (double) imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
    }

    public void update() {
        updatePoseEstimate();
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        frontLeft.setPower(v);
        backLeft.setPower(v1);
        backRight.setPower(v2);
        frontRight.setPower(v3);
    }
}