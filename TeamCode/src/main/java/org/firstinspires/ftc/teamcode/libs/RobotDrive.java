package org.firstinspires.ftc.teamcode.libs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.Parameters;

public class RobotDrive {
    public HardwareMap hardwareMap;
    public DcMotor backLeftMotor, frontLeftMotor, backRightMotor, frontRightMotor;
    private double m_rightSideInvertMultiplier = -1.0;

    double[] wheels = new double[4];
    /***
     * Initializes drive motors
     */
    public RobotDrive(HardwareMap hardwareMap, String backLeftName, String frontLeftName,
                      String backRightName, String frontRightName, boolean reverse) {
        backLeftMotor = hardwareMap.dcMotor.get(backLeftName);
        frontLeftMotor = hardwareMap.dcMotor.get(frontLeftName);
        backRightMotor = hardwareMap.dcMotor.get(backRightName);
        frontRightMotor = hardwareMap.dcMotor.get(frontRightName);
        this.hardwareMap = hardwareMap;


        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);



    }

    public void arcadeDrive(double moveValue, double rotateValue, boolean squareInputs) {
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

        if(squareInputs) {
            setLeftDrive(Math.abs(left) * left);
            setRightDrive(Math.abs(right) * right);
        }
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

    public void setLeftDrive(double speed) {
        frontLeftMotor.setPower(speed);
        backLeftMotor.setPower(speed);
    }


    public void setRightDrive(double speed) {
        frontRightMotor.setPower(speed);
        backRightMotor.setPower(speed);
    }

    public double getAverageDistance() {
        return (backLeftMotor.getCurrentPosition()* Parameters.kInchesPerTick + backRightMotor.getCurrentPosition()*Parameters.kInchesPerTick) / 2;
    }

    public double getLeftDistance() {
        return (backLeftMotor.getCurrentPosition()*Parameters.kInchesPerTick);
    }

    public double getRightDistance() {
        return (backRightMotor.getCurrentPosition()*Parameters.kInchesPerTick);
    }

    public double limit(double number){
        if(number < -1.0){
            return -1.0;
        } else if(number > 1){
            return 1;
        } else {
            return number;
        }
    }

    public void resetEncoders() {
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void positionEncoders() {
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setDistance(DcMotor motor, double distance) {
        motor.setTargetPosition((int) (distance * Parameters.kTickPerInches));
    }

    public void setLeftRightCornertDistance(double distance) {
        backLeftMotor.setTargetPosition((int) (distance * Parameters.kTickPerInches));
        frontRightMotor.setTargetPosition((int) (distance * Parameters.kTickPerInches));
    }

    public void setRightLeftCornertDistance(double distance) {
        frontLeftMotor.setTargetPosition((int) (distance * Parameters.kTickPerInches));
        backRightMotor.setTargetPosition((int) (distance * Parameters.kTickPerInches));
    }

    public void setLeftRightCornertPower(double speed) {
        backLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
    }

    public void setRightLeftCornertPower(double speed) {
        frontLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);
    }

    public void stopDriving() {
        setLeftDrive(0);
        setRightDrive(0);
    }



}