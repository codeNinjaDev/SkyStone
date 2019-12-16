package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Parameters;
import org.firstinspires.ftc.teamcode.libs.PIDController;
import org.firstinspires.ftc.teamcode.libs.RobotDrive;
import org.firstinspires.ftc.teamcode.libs.Vector2D;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class StrafeCommand implements Command {
    /***
     * PID Controllers for the corner wheels and the heading correction
     */
    private PIDController forwardSlashController, backSlashController, headingController;

    private DriveSubsystem driveSubsystem;
    private double targetForwardSlashPosition, targetBackSlashPosition, xDistance, yDistance, timeout;
    private ElapsedTime timer;
    boolean noGyroCorrection = false;
    boolean distanceReached;

    public StrafeCommand(DriveSubsystem driveSubsystem, double targetDistance, double maxSpeed, double timeout) {
        this.driveSubsystem = driveSubsystem;


        targetForwardSlashPosition = targetDistance;
        targetBackSlashPosition = -targetDistance;
        forwardSlashController = new PIDController(0.04, 0.00001, 0.001, maxSpeed, 1);
        backSlashController = new PIDController(0.04, 0.00001, 0.001, maxSpeed, 1);
        headingController = new PIDController(0.03, 0, 0, 0.3, 3);

        timer = new ElapsedTime();
        this.timeout = timeout;
    }

    public void init() {

        driveSubsystem.reset();
        distanceReached = false;

        timer.reset();
        timer.startTime();

        forwardSlashController.setSetpoint(targetForwardSlashPosition, true);
        backSlashController.setSetpoint(targetBackSlashPosition, true);
        headingController.setSetpoint(0, true);
    }

    public void update(Telemetry tl) {
        double rotate = -headingController.run(driveSubsystem.getHeading());
        double averageFSlashPosition = (RobotDrive.getDistance(driveSubsystem.robotDrive.backLeftMotor)
                +  RobotDrive.getDistance(driveSubsystem.robotDrive.frontRightMotor)) / 2.0;
        double averageBSlashPosition = (RobotDrive.getDistance(driveSubsystem.robotDrive.backRightMotor)
                +  RobotDrive.getDistance(driveSubsystem.robotDrive.frontLeftMotor)) / 2.0;

        double fSlashOutput = forwardSlashController.run(averageFSlashPosition);
        double bSlashOutput = backSlashController.run(averageBSlashPosition);

        if(!noGyroCorrection) {
            if (!distanceReached) {
                RobotDrive.setMotorVelocity(driveSubsystem.robotDrive.backLeftMotor, bSlashOutput + rotate, true);
                RobotDrive.setMotorVelocity(driveSubsystem.robotDrive.frontLeftMotor, fSlashOutput + rotate, true);
                RobotDrive.setMotorVelocity(driveSubsystem.robotDrive.backRightMotor, fSlashOutput - rotate, true);
                RobotDrive.setMotorVelocity(driveSubsystem.robotDrive.frontRightMotor, bSlashOutput - rotate, true);
            } else {
                RobotDrive.setMotorVelocity(driveSubsystem.robotDrive.backLeftMotor, rotate, true);
                RobotDrive.setMotorVelocity(driveSubsystem.robotDrive.frontLeftMotor, rotate, true);
                RobotDrive.setMotorVelocity(driveSubsystem.robotDrive.backRightMotor, -rotate, true);
                RobotDrive.setMotorVelocity(driveSubsystem.robotDrive.frontRightMotor, -rotate, true);
            }
        } else {
            RobotDrive.setMotorVelocity(driveSubsystem.robotDrive.backLeftMotor, bSlashOutput, true);
            RobotDrive.setMotorVelocity(driveSubsystem.robotDrive.frontLeftMotor, fSlashOutput, true);
            RobotDrive.setMotorVelocity(driveSubsystem.robotDrive.backRightMotor, fSlashOutput, true);
            RobotDrive.setMotorVelocity(driveSubsystem.robotDrive.frontRightMotor, bSlashOutput, true);
        }

    }

    public boolean isFinished() {

        distanceReached = forwardSlashController.onTarget() && backSlashController.onTarget();

        boolean timeoutReached = timer.seconds() >= timeout;
        boolean gyro = headingController.onTarget();
        if(noGyroCorrection) {
            return (distanceReached)|| timeoutReached;
        } else {
            return (distanceReached && gyro)|| timeoutReached;
        }
    }

    public void finish() {
        driveSubsystem.stop();
        driveSubsystem.reset();
    }
}
