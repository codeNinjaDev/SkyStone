package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Parameters;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.ArrayList;

public class MecanumDriveCommand implements Command {
    private DriveSubsystem driveSubsystem;
    private double targetDistance, theta, yaw;
    private double y_distance, x_distance;
    private ArrayList<Double> wheelTargets;
    private ElapsedTime timer;
    private double timeout, speed, x_speed, y_speed;
    Telemetry telemetry;

    public MecanumDriveCommand(DriveSubsystem driveSubsystem, double targetDistance, double theta, double timeout, Telemetry telemetry) {
        this.driveSubsystem = driveSubsystem;
        this.targetDistance = targetDistance;
        this.theta = -theta;

        this.y_distance = Math.sin(Math.toRadians(-theta)) * targetDistance;
        this.x_distance = Math.cos(Math.toRadians(-theta)) * targetDistance;
        this.timeout = timeout;
        this.telemetry = telemetry;
        wheelTargets = new ArrayList<Double>();

        wheelTargets.add(y_distance + x_distance); // + 1.366 (13.66)
        wheelTargets.add(y_distance - x_distance); // - .366 (3.66)
        wheelTargets.add(y_distance - x_distance); // - .366 (3.66)
        wheelTargets.add(y_distance + x_distance); // + 1.366 (13.66)

        telemetry.addData("BackLeftWheel Desired: ", wheelTargets.get(0));
        telemetry.addData("FrontLeftWheel Desired: ", wheelTargets.get(1));
        telemetry.addData("BackRightWheel Desired: ", wheelTargets.get(2));
        telemetry.addData("FrontRightWheel Desired: ", wheelTargets.get(3));

        timer = new ElapsedTime();
        this.speed = 0.6;
    }

    public MecanumDriveCommand(DriveSubsystem driveSubsystem, double targetDistance, double theta, double speed, double timeout, Telemetry telemetry) {
        this.driveSubsystem = driveSubsystem;
        this.targetDistance = targetDistance;
        this.theta = -theta;

        this.y_distance = Math.sin(Math.toRadians(-theta)) * targetDistance;
        this.x_distance = Math.cos(Math.toRadians(-theta)) * targetDistance;
        this.timeout = timeout;
        this.telemetry = telemetry;
        wheelTargets = new ArrayList<Double>();

        wheelTargets.add(y_distance + x_distance); // + 1.366 (13.66)
        wheelTargets.add(y_distance - x_distance); // - .366 (3.66)
        wheelTargets.add(y_distance - x_distance); // - .366 (3.66)
        wheelTargets.add(y_distance + x_distance); // + 1.366 (13.66)

        telemetry.addData("BackLeftWheel Desired: ", wheelTargets.get(0));
        telemetry.addData("FrontLeftWheel Desired: ", wheelTargets.get(1));
        telemetry.addData("BackRightWheel Desired: ", wheelTargets.get(2));
        telemetry.addData("FrontRightWheel Desired: ", wheelTargets.get(3));

        timer = new ElapsedTime();
        this.speed = speed;
    }

    public MecanumDriveCommand(DriveSubsystem driveSubsystem, double x_distance, double y_distance, double speed, double timeout) {
        this.driveSubsystem = driveSubsystem;

        this.y_distance = y_distance;
        this.x_distance = x_distance;
        this.timeout = timeout;
        wheelTargets = new ArrayList<Double>();

        wheelTargets.add(y_distance + x_distance); // + 1.366 (13.66)
        wheelTargets.add(y_distance - x_distance); // - .366 (3.66)
        wheelTargets.add(y_distance - x_distance); // - .366 (3.66)
        wheelTargets.add(y_distance + x_distance); // + 1.366 (13.66)


        timer = new ElapsedTime();
        this.speed = speed;
    }
    /*** Configures everything ***/
    public void init() {
        timer.reset();
        timer.startTime();
        driveSubsystem.robotDrive.resetEncoders();
        driveSubsystem.robotDrive.setDistance(driveSubsystem.robotDrive.backLeftMotor, wheelTargets.get(0));
        driveSubsystem.robotDrive.setDistance(driveSubsystem.robotDrive.frontLeftMotor, wheelTargets.get(1));
        driveSubsystem.robotDrive.setDistance(driveSubsystem.robotDrive.backRightMotor, wheelTargets.get(2));
        driveSubsystem.robotDrive.setDistance(driveSubsystem.robotDrive.frontRightMotor, wheelTargets.get(3));
        driveSubsystem.robotDrive.positionEncoders();


        x_speed = (speed * x_distance) / (Math.abs(x_distance) + Math.abs(y_distance));
        y_speed = (speed * y_distance) / (Math.abs(x_distance) + Math.abs(y_distance));

    }
    /*** Runs in a loop ***/
    public void update(Telemetry tl) {

        driveSubsystem.robotDrive.frontRightMotor.setPower(speed);
        driveSubsystem.robotDrive.frontLeftMotor.setPower(speed);
        driveSubsystem.robotDrive.backLeftMotor.setPower(speed);
        driveSubsystem.robotDrive.backRightMotor.setPower(speed);


    }

    /*** Checks if command is finished ***/
    public boolean isFinished() {

        boolean distanceReached = !driveSubsystem.robotDrive.backLeftMotor.isBusy()
                && !driveSubsystem.robotDrive.frontLeftMotor.isBusy()
                && !driveSubsystem.robotDrive.backRightMotor.isBusy()
                && !driveSubsystem.robotDrive.frontRightMotor.isBusy();

        boolean timeoutReached = timer.seconds() >= timeout;

        return distanceReached || timeoutReached;
    }
    /*** Runs when command is finished ***/
    public void finish() {

        driveSubsystem.robotDrive.resetEncoders();

    }
}
