package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Parameters;
import org.firstinspires.ftc.teamcode.libs.PIDController;
import org.firstinspires.ftc.teamcode.libs.RobotDrive;
import org.firstinspires.ftc.teamcode.libs.Vector2D;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.ArrayList;

public class MecanumDriveCommand implements Command {
    private DriveSubsystem driveSubsystem;
    Vector2D positionVector, velocityVector;

    PIDController gyroPID;
    private ElapsedTime timer;
    private double timeout;
    Telemetry telemetry;

    public MecanumDriveCommand(DriveSubsystem driveSubsystem, double targetDistance, double theta, double timeout, Telemetry telemetry) {



        this.timeout = timeout;
        this.telemetry = telemetry;
        this.positionVector = Vector2D.polarVector(targetDistance, theta);

        timer = new ElapsedTime();
        double defaultSpeed = 24;
        velocityVector = Vector2D.polarVector(defaultSpeed, theta);
    }

    public MecanumDriveCommand(DriveSubsystem driveSubsystem, double targetDistance, double theta, double speed, double timeout, Telemetry telemetry) {
        this.driveSubsystem = driveSubsystem;

        this.timeout = timeout;
        this.telemetry = telemetry;

        this.positionVector = Vector2D.polarVector(targetDistance, theta);
        velocityVector = Vector2D.polarVector(speed, theta);

        timer = new ElapsedTime();
    }

    public MecanumDriveCommand(DriveSubsystem driveSubsystem, double x_distance, double y_distance, double speed, double timeout) {
        this.driveSubsystem = driveSubsystem;
        this.timeout = timeout;
        timer = new ElapsedTime();
        this.positionVector = new Vector2D(x_distance, y_distance);
        velocityVector = Vector2D.polarVector(speed, Math.atan2(x_distance, y_distance));


    }
    /*** Configures everything ***/
    public void init() {
        timer.reset();
        timer.startTime();
        driveSubsystem.reset();

        gyroPID = new PIDController(0.08, 0.00001, 0.001, 9, 5);
        gyroPID.setSetpoint(0);

        positionVector.rotate(-45);
        positionVector.scale(Math.sqrt(2));

        // Shift the coordinate plane 45 degrees counter clockwise -  Linear Transformation
        velocityVector.rotate(-45);
        //Because magnitude of [1, 1] vector is sqrt(2)
        velocityVector.scale(Math.sqrt(2));


    }
    /*** Runs in a loop ***/
    public void update(Telemetry tl) {
        double rotate = gyroPID.run(driveSubsystem.getHeading());
        RobotDrive.setMotorVelocity(driveSubsystem.robotDrive.backLeftMotor, velocityVector.x + rotate, true);
        RobotDrive.setMotorVelocity(driveSubsystem.robotDrive.frontLeftMotor, velocityVector.y + rotate, true);
        RobotDrive.setMotorVelocity(driveSubsystem.robotDrive.backRightMotor, velocityVector.y - rotate, true);
        RobotDrive.setMotorVelocity(driveSubsystem.robotDrive.frontRightMotor, velocityVector.x - rotate, true);
    }

    /*** Checks if command is finished ***/
    public boolean isFinished() {

        boolean fSlashComplete = Math.abs(driveSubsystem.robotDrive.getFSlashAverageDistance()) >= Math.abs(positionVector.x);
        boolean bSlashComplete = Math.abs(driveSubsystem.robotDrive.getBSlashAverageDistance()) >= Math.abs(positionVector.y);

        boolean distanceReached = fSlashComplete && bSlashComplete;

        boolean timeoutReached = timer.seconds() >= timeout;

        return distanceReached || timeoutReached;
    }
    /*** Runs when command is finished ***/
    public void finish() {

        driveSubsystem.robotDrive.resetEncoders();

    }
}
