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
    public StrafeCommand(DriveSubsystem driveSubsystem, double targetDistance, double maxSpeed, double timeout) {
        this.driveSubsystem = driveSubsystem;


        targetForwardSlashPosition = -targetDistance;
        targetBackSlashPosition = targetDistance;
        PIDFCoefficients drivePID = driveSubsystem.robotDrive.getDriveMotorCoefficients();
        forwardSlashController = new PIDController(drivePID.p * Parameters.kTickPerInches, drivePID.i, drivePID.d, maxSpeed, 5);
        backSlashController = new PIDController(drivePID.p * Parameters.kTickPerInches, drivePID.i, drivePID.d, maxSpeed, 5);
        headingController = new PIDController(0.001, 0, 0, 0.3, 5);

        timer = new ElapsedTime();
        this.timeout = timeout;
    }

    public void init() {
        forwardSlashController.setSetpoint(targetForwardSlashPosition);
        backSlashController.setSetpoint(targetBackSlashPosition);
        headingController.setSetpoint(0);
        driveSubsystem.reset();
        timer.reset();
        timer.startTime();
    }

    public void update(Telemetry tl) {
        double rotate = headingController.run(driveSubsystem.getHeading());
        double averageFSlashPosition = (RobotDrive.getDistance(driveSubsystem.robotDrive.backLeftMotor)
                +  RobotDrive.getDistance(driveSubsystem.robotDrive.frontRightMotor)) / 2.0;
        double averageBSlashPosition = (RobotDrive.getDistance(driveSubsystem.robotDrive.backRightMotor)
                +  RobotDrive.getDistance(driveSubsystem.robotDrive.frontLeftMotor)) / 2.0;

        double fSlashOutput = forwardSlashController.run(averageFSlashPosition);
        double bSlashOutput = backSlashController.run(-averageBSlashPosition);

        driveSubsystem.robotDrive.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveSubsystem.robotDrive.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveSubsystem.robotDrive.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveSubsystem.robotDrive.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveSubsystem.robotDrive.frontLeftMotor.setPower(bSlashOutput + rotate);
        driveSubsystem.robotDrive.backLeftMotor.setPower(fSlashOutput + rotate);
        driveSubsystem.robotDrive.frontRightMotor.setPower(fSlashOutput - rotate);
        driveSubsystem.robotDrive.backRightMotor.setPower(bSlashOutput - rotate);

    }

    public boolean isFinished() {
        boolean reachedTimeout = timer.startTime() > timeout;
        boolean reachedDistance = backSlashController.onTarget() && forwardSlashController.onTarget();

        return  reachedTimeout || reachedDistance;
    }

    public void finish() {
        driveSubsystem.stop();
        driveSubsystem.reset();
    }
}
