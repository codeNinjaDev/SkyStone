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
    private double timeout;


    public MecanumDriveCommand(DriveSubsystem driveSubsystem, double targetDistance, double theta, double timeout, Telemetry telemetry) {
        this.driveSubsystem = driveSubsystem;
        this.targetDistance = targetDistance;
        this.theta = -theta;

        this.y_distance = Math.sin(Math.toRadians(-theta)) * targetDistance;
        this.x_distance = Math.cos(Math.toRadians(-theta)) * targetDistance;
        this.timeout = timeout;

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

    }

    /*** Configures everything ***/
    public void init() {
        driveSubsystem.robotDrive.resetEncoders();
        driveSubsystem.robotDrive.setDistance(driveSubsystem.robotDrive.backLeftMotor, wheelTargets.get(0));
        driveSubsystem.robotDrive.setDistance(driveSubsystem.robotDrive.frontLeftMotor, wheelTargets.get(1));
        driveSubsystem.robotDrive.setDistance(driveSubsystem.robotDrive.backRightMotor, wheelTargets.get(2));
        driveSubsystem.robotDrive.setDistance(driveSubsystem.robotDrive.frontRightMotor, wheelTargets.get(3));
        driveSubsystem.robotDrive.positionEncoders();


    }
    /*** Runs in a loop ***/
    public void update(Telemetry tl) {

        driveSubsystem.robotDrive.frontRightMotor.setPower(0.7);
        driveSubsystem.robotDrive.frontLeftMotor.setPower(0.7);
        driveSubsystem.robotDrive.backLeftMotor.setPower(0.7);
        driveSubsystem.robotDrive.backRightMotor.setPower(0.7);
        tl.addData("BackLeftWheel Desired: ", wheelTargets.get(0));
        tl.addData("FrontLeftWheel Desired: ", wheelTargets.get(1));
        tl.addData("BackRightWheel Desired: ", wheelTargets.get(2));
        tl.addData("FrontRightWheel Desired: ", wheelTargets.get(3));
        tl.addData("BackLeftWheel Direction: ", driveSubsystem.robotDrive.backLeftMotor.getDirection());
        tl.addData("BackRightWheel Direction: ", driveSubsystem.robotDrive.backRightMotor.getDirection());
        tl.addData("frontLeftWheel Direction: ", driveSubsystem.robotDrive.frontLeftMotor.getDirection());
        tl.addData("FrontRightWheel Direction: ", driveSubsystem.robotDrive.frontRightMotor.getDirection());
    }

    /*** Checks if command is finished ***/
    public boolean isFinished() {
        /*boolean backLeft = (Math.abs((driveSubsystem.robotDrive.backLeftMotor.getCurrentPosition()
                * (Parameters.kInchesPerTick) - wheelTargets.get(0))) < 1);
        boolean frontLeft = (Math.abs((driveSubsystem.robotDrive.frontLeftMotor.getCurrentPosition()
                * (Parameters.kInchesPerTick) - wheelTargets.get(1))) < 1);
        boolean backRight = (Math.abs((driveSubsystem.robotDrive.frontLeftMotor.getCurrentPosition()
                * (Parameters.kInchesPerTick) - wheelTargets.get(2))) < 1);
        boolean frontRight = (Math.abs((driveSubsystem.robotDrive.frontLeftMotor.getCurrentPosition()
                * (Parameters.kInchesPerTick) - wheelTargets.get(3))) < 1);
        boolean distanceReached = backLeft && frontLeft && backRight && frontRight;*/
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
