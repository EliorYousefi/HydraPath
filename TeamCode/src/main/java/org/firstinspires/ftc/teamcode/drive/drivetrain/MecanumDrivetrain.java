package org.firstinspires.ftc.teamcode.drive.drivetrain;

import static java.lang.Math.atan2;
import static java.lang.Math.hypot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.util.Pose;

@Config
public class MecanumDrivetrain implements Drivetrain {

    private RobotHardware robot;
    public DcMotorEx frontLeft, backLeft, backRight, frontRight;

    public MecanumDrivetrain(RobotHardware robot) {
        this.robot = robot;

        frontLeft = robot.frontLeftMotor;
        backLeft = robot.backLeftMotor;
        backRight = robot.backRightMotor;
        frontRight = robot.frontRightMotor;
    }


    @Override
    public void set(Pose pose, double maxPower)
    {
        frontLeft.setPower(Range.clip(pose.x + pose.y + pose.heading, -maxPower, maxPower));
        backLeft.setPower(Range.clip(pose.x - pose.y + pose.heading, -maxPower, maxPower));
        frontRight.setPower(Range.clip(pose.x - pose.y - pose.heading, -maxPower, maxPower));
        backRight.setPower(Range.clip(pose.x + pose.y - pose.heading, -maxPower, maxPower));
    }


}