package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Parameters;

import org.firstinspires.ftc.teamcode.libs.TriggerReader;
import org.firstinspires.ftc.teamcode.libs.DriveUtils;
import org.firstinspires.ftc.teamcode.libs.GamepadKeys;
import org.firstinspires.ftc.teamcode.libs.PIDController;
import org.firstinspires.ftc.teamcode.libs.RobotDrive;
import org.firstinspires.ftc.teamcode.libs.SuperGamepad;
import org.firstinspires.ftc.teamcode.libs.Vector2D;

/**
 * Created by peter on 11/22/17.
 */

public class DriveSubsystem implements Subsystem {
    private SuperGamepad driverGamepad;
    double MAX_SPEED = 1;
    private Telemetry tl;
    public HardwareMap hardwareMap = null;

    private TriggerReader slowModeButton;
    private
    //Drive Variables
    double drive;
    double strafe;
    double rotate;

    double front_left;
    double rear_left;
    double front_right;
    double rear_right;

    //Speed Variables
    boolean fastMode = true;


    // Direction Variables
    int direction = 1;
    boolean toggleState = false;
    boolean buttonState = false;
    double yaw;
    // The IMU sensor object
    public BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    ElapsedTime pidTimer;
    public RobotDrive robotDrive;
    //TODO initialize frontMOtors
    public DriveSubsystem(HardwareMap hardwareMap, SuperGamepad driverGamepad, Telemetry tl) {
        pidTimer = new ElapsedTime();
        this.driverGamepad = driverGamepad;
        this.tl = tl;
        this.hardwareMap = hardwareMap;
        this.yaw = Double.MAX_VALUE;

        slowModeButton = new TriggerReader(driverGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER);

        robotDrive = new RobotDrive(hardwareMap, "rearLeft",
                "frontLeft", "rearRight", "frontRight", false);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }

    public void init() {
        robotDrive.setBrakeMode();
    }

    public double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }
    public void update() {
        slowModeButton.readValue();

        //speed

        if(slowModeButton.isDown()){
            MAX_SPEED = 0.25;
        } else {
            MAX_SPEED = 1;
        }

        strafe = driverGamepad.getLeftX();
        if(Math.abs(strafe) < 0.5) {
            strafe = 0;
        }

        drive = driverGamepad.getLeftY();
        rotate = driverGamepad.getRightX();

        //Mecanum direction calculation

        // If direction is normal
        front_left = drive + strafe + rotate;
        rear_left = drive - strafe + rotate;
        front_right = drive - strafe - rotate;
        rear_right = drive + strafe - rotate;

        robotDrive.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotDrive.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotDrive.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotDrive.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robotDrive.frontLeftMotor.setPower(robotDrive.limit(front_left)* MAX_SPEED);
        robotDrive.backLeftMotor.setPower(robotDrive.limit(rear_left)* MAX_SPEED);
        robotDrive.frontRightMotor.setPower(robotDrive.limit(front_right)* MAX_SPEED);
        robotDrive.backRightMotor.setPower(robotDrive.limit(rear_right)* MAX_SPEED);

    }

    public void arcadeDrive(double forward, double rotate, boolean squareInputs) {
        robotDrive.arcadeDrive(forward, rotate, squareInputs);
    }


    public void reset() {
        imu.getAngularOrientation().firstAngle = 0;
        robotDrive.resetEncoders();
    }



    public void stop() {
        robotDrive.stopDriving();
        reset();
    }


    // Computes the current battery voltage
    public double getBatteryVoltage() {
        try {
            double result = Double.POSITIVE_INFINITY;
            for (VoltageSensor sensor : hardwareMap.voltageSensor) {
                double voltage = sensor.getVoltage();
                if (voltage > 0) {
                    result = Math.min(result, voltage);
                }
            }
            tl.addData("Voltage ", result);
            tl.update();
            return result;
        } catch(Exception e) {
            tl.addData("DriveException ", e);
            tl.update();
            return 12;
        }
    }
}
