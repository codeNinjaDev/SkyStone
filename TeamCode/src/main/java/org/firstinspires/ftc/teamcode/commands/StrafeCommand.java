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
        PIDFCoefficients drivePID = driveSubsystem.robotDrive.getDriveMotorCoefficients();
        forwardSlashController = new PIDController(drivePID.p * Parameters.kTickPerInches, drivePID.i, drivePID.d, maxSpeed, 5);
        backSlashController = new PIDController(drivePID.p * Parameters.kTickPerInches, drivePID.i, drivePID.d, maxSpeed, 5);
        headingController = new PIDController(0.03, 0, 0, 0.3, 5);

        timer = new ElapsedTime();
        this.timeout = timeout;
    }

    public void init() {
        forwardSlashController.setSetpoint(targetForwardSlashPosition);
        backSlashController.setSetpoint(targetBackSlashPosition);
        headingController.setSetpoint(0);
        driveSubsystem.reset();
        distanceReached = false;

        timer.reset();
        timer.startTime();
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
