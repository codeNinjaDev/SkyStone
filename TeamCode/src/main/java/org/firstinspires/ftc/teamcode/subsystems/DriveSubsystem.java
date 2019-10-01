package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Parameters;

import org.firstinspires.ftc.teamcode.libs.DriveUtils;
import org.firstinspires.ftc.teamcode.libs.Vector2D;

/**
 * Created by peter on 11/22/17.
 */

public class DriveSubsystem implements Subsystem {
    private Gamepad driverGamepad;
    double MAX_SPEED = 1;
    private double m_rightSideInvertMultiplier = -1.0;

    private Telemetry tl;
    public HardwareMap hardwareMap = null;

    public DcMotor backLeftMotor = null;
    public DcMotor backRightMotor = null;

    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    ElapsedTime pidTimer;

    //TODO initialize frontMOtors
    public DriveSubsystem(HardwareMap hardwareMap, Gamepad driverGamepad, Telemetry tl) {
        pidTimer = new ElapsedTime();
        this.driverGamepad = driverGamepad;
        this.tl = tl;
        this.hardwareMap = hardwareMap;
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");


        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);


        resetEncoders();

    }

    public void init() {
        //resetEncoders();
    }

    public void arcadeDrive(double moveValue, double rotateValue) {
        double left = moveValue + rotateValue;
        double right = moveValue - rotateValue;

        if(left > 1) {
            left = 1;
        } else if(left < -1) {
            left = -1;
        }

        if(right > 1) {
            right = 1;
        } else if(right < -1) {
            right = -1;
        }
        tl.addData("ArcadeDriveLeftPower", Math.abs(left)*left*MAX_SPEED);
        tl.addData("ArcadeDriveRightPower", Math.abs(right)*right*MAX_SPEED);
        tl.addData("LeftMotor", backLeftMotor);
        tl.addData("LeftMotorPower", backLeftMotor.getPower());
        tl.addData("LeftMotorPort", backLeftMotor.getPortNumber());
        tl.addData("rightMotorPort", backRightMotor.getPortNumber());


        setLeftDrive(Math.abs(left)*left*MAX_SPEED);
        setRightDrive(Math.abs(right)*right*MAX_SPEED);
    }

    /**
     * Drive method for Mecanum platform.
     *
     * <p>Angles are measured clockwise from the positive X axis. The robot's speed is independent
     * from its angle or rotation rate.
     *
     * @param ySpeed    The robot's speed along the Y axis [-1.0..1.0]. Right is positive.
     * @param xSpeed    The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
     * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
     *                  positive.
     * @param gyroAngle The current angle reading from the gyro in degrees around the Z axis. Use
     *                  this to implement field-oriented controls.
     */
    public void driveCartesian(double ySpeed, double xSpeed, double zRotation, double gyroAngle) {
        ySpeed = DriveUtils.clamp(ySpeed, -1.0, 1.0);
        ySpeed = DriveUtils.applyDeadband(ySpeed, 0.01);

        xSpeed = DriveUtils.clamp(xSpeed, -1.0, 1.0);
        xSpeed = DriveUtils.applyDeadband(xSpeed, 0.01);

        Vector2D input = new Vector2D(ySpeed, xSpeed);
        input.rotate(-gyroAngle);

        double[] wheelSpeeds = new double[4];
        wheelSpeeds[0] = input.x + input.y + zRotation;
        wheelSpeeds[1] = -input.x + input.y - zRotation;
        wheelSpeeds[2] = -input.x + input.y + zRotation;
        wheelSpeeds[3] = input.x + input.y - zRotation;

        DriveUtils.normalize(wheelSpeeds);

        frontLeftMotor.setPower(wheelSpeeds[0]);
        frontRightMotor.setPower(wheelSpeeds[1] * m_rightSideInvertMultiplier);
        backLeftMotor.setPower(wheelSpeeds[2]);
        backRightMotor.setPower(wheelSpeeds[0]  * m_rightSideInvertMultiplier);


    }

    /**
     * Drive method for Mecanum platform.
     *
     * <p>Angles are measured counter-clockwise from straight ahead. The speed at which the robot
     * drives (translation) is independent from its angle or rotation rate.
     *
     * @param magnitude The robot's speed at a given angle [-1.0..1.0]. Forward is positive.
     * @param angle     The angle around the Z axis at which the robot drives in degrees [-180..180].
     * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
     *                  positive.
     */
    public void drivePolar(double magnitude, double angle, double zRotation) {

        driveCartesian(magnitude * Math.sin(angle * (Math.PI / 180.0)),
                magnitude * Math.cos(angle * (Math.PI / 180.0)), zRotation, 0.0);
    }

    public void tankDrive(double left, double right) {

        if(left > 1) {
            left = 1;
        } else if(left < -1) {
            left = -1;
        }

        if(right > 1) {
            right = 1;
        } else if(right < -1) {
            right = -1;
        }
        setLeftDrive(left);
        setRightDrive(right);
    }

    public double getAverageDistance() {
        return (backLeftMotor.getCurrentPosition()*Parameters.kInchesPerTick + backRightMotor.getCurrentPosition()*Parameters.kInchesPerTick) / 2;
    }
    
    public double getLeftDistance() {
        return (backLeftMotor.getCurrentPosition()*Parameters.kInchesPerTick);
    }
    
    public double getRightDistance() {
        return (backRightMotor.getCurrentPosition()*Parameters.kInchesPerTick);
    }
    public void update(Telemetry telemetry) {
        // If RT is pressed slow down
        if(driverGamepad.right_trigger > 0.2) {
            MAX_SPEED = .7;
        } else {
            MAX_SPEED = 1;
        }
        arcadeDrive(-driverGamepad.left_stick_y, driverGamepad.right_stick_x);
        //arcadeDrive(humanControl.getDriverRightJoyX(), humanControl.getDriverLeftJoyY());
    }

    public void resetEncoders() {

        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public void reset() {
        resetEncoders();




    }
    public void driveForward(double power) {

        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
    }

    public void setRightDrive(double power) {
        backRightMotor.setPower(power);
    }
    public void setLeftDrive(double power) {
        backLeftMotor.setPower(power);
    }

    public void stopDriving() {

        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    public void stop() {
        stopDriving();
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
