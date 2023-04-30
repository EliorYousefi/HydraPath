package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import javax.annotation.concurrent.GuardedBy;

public class RobotHardware {

    public DcMotorEx frontLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backLeftMotor;
    public DcMotorEx backRightMotor;

    public Motor.Encoder parallelPod;
    public Motor.Encoder leftPerpindicularPod;
    public Motor.Encoder rightPerpindicularPod;

    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    public BNO055IMU imu;
    private Thread imuThread;
    private double imuAngle = 0;
    private double imuOffset = 0;

    private static RobotHardware instance = null;

    public boolean enabled;

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        if (Globals.USING_IMU) {
            synchronized (imuLock) {
                imu = hardwareMap.get(BNO055IMU.class, "imu");
                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
                imu.initialize(parameters);
            }
        }

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");


        parallelPod = new MotorEx(hardwareMap, "parallelPod").encoder;
        leftPerpindicularPod = new MotorEx(hardwareMap, "leftPerpindicularPod").encoder;
        rightPerpindicularPod = new MotorEx(hardwareMap, "rightPerpindicularPod").encoder;

    }

    public void reset() {
        try {
            leftPerpindicularPod.reset();
            rightPerpindicularPod.reset();
            parallelPod.reset();
        } catch (Exception e) {}
        imuOffset = imu.getAngularOrientation().firstAngle;
    }

    public double getAngle() {
        return imuAngle - imuOffset;
    }

    public void startIMUThread(LinearOpMode opMode) {
        if (Globals.USING_IMU) {
            imuThread = new Thread(() -> {
                while (!opMode.isStopRequested() && opMode.opModeIsActive()) {
                    synchronized (imuLock) {
                        imuAngle = imu.getAngularOrientation().firstAngle;
                    }
                }
            });
            imuThread.start();
        }
    }
}
