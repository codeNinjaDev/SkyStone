package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.PIDController;
import org.firstinspires.ftc.teamcode.libs.RobotDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class DrivePIDCommand implements Command {

    DriveSubsystem driveSubsystem;
    private ElapsedTime timer;
    double distance, timeout, speed;

    public DrivePIDCommand(DriveSubsystem driveSubsystem, double distance, double speed, double timeout) {
        this.driveSubsystem = driveSubsystem;
        this.distance = distance;
        this.timeout = timeout;
        this.speed = speed;
        timer = new ElapsedTime();
    }

    public void init() {
        driveSubsystem.reset();
        timer.reset();
        timer.startTime();
        RobotDrive.setDistance(driveSubsystem.robotDrive.backLeftMotor, distance);
        RobotDrive.setDistance(driveSubsystem.robotDrive.frontLeftMotor, distance);
        RobotDrive.setDistance(driveSubsystem.robotDrive.backRightMotor, distance);
        RobotDrive.setDistance(driveSubsystem.robotDrive.frontRightMotor, distance);


    }

    public void update(Telemetry tl) {
        driveSubsystem.robotDrive.backLeftMotor.setPower(speed);
        driveSubsystem.robotDrive.backRightMotor.setPower(speed);
        driveSubsystem.robotDrive.frontLeftMotor.setPower(speed);
        driveSubsystem.robotDrive.frontRightMotor.setPower(speed);

    }

    @Override
    public boolean isFinished() {
        boolean distanceReached = !driveSubsystem.robotDrive.backLeftMotor.isBusy() && !driveSubsystem.robotDrive.backRightMotor.isBusy() && !driveSubsystem.robotDrive.frontLeftMotor.isBusy() && !driveSubsystem.robotDrive.frontRightMotor.isBusy();
        boolean timeReached = timer.seconds() >= timeout;
        return distanceReached || timeReached;
    }

    @Override
    public void finish() {
        driveSubsystem.stop();
        driveSubsystem.reset();
    }
}
