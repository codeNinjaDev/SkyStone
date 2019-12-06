package org.firstinspires.ftc.teamcode.commands;
/*** Allows sequential actions in autonomous ***/
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Parameters;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.ArrayList;

public class DrivePolarCommand implements Command{
    private DriveSubsystem driveSubsystem;
    private double targetDistance, theta, yaw;
    private double y_distance, x_distance;
    private ArrayList<Double> wheelTargets;
    private ElapsedTime timer;
    private double timeout;


    public DrivePolarCommand(DriveSubsystem driveSubsystem, double targetDistance, double theta, double timeout, Telemetry telemetry) {
        this.driveSubsystem = driveSubsystem;
        this.targetDistance = targetDistance;
        this.theta = theta;
        this.yaw = yaw;

        this.y_distance = Math.sin(Math.toRadians(theta)) * targetDistance;
        this.x_distance = Math.cos(Math.toRadians(theta)) * targetDistance;
        this.timeout = timeout;

        wheelTargets = new ArrayList<Double>();

        wheelTargets.add(y_distance + x_distance);
        wheelTargets.add(y_distance - x_distance);
        wheelTargets.add(y_distance - x_distance);
        wheelTargets.add(y_distance + x_distance);
        timer = new ElapsedTime();

    }

    /*** Configures everything ***/
    public void init() {
        driveSubsystem.robotDrive.resetEncoders();

    }
    /*** Runs in a loop ***/
    public void update(Telemetry tl) {
        driveSubsystem.robotDrive.drivePolar(.7, theta, 0);
    }
    /*** Checks if command is finished ***/
    public boolean isFinished() {
        boolean backLeft = (Math.abs((driveSubsystem.robotDrive.backLeftMotor.getCurrentPosition()
                * (Parameters.kInchesPerTick) - wheelTargets.get(0))) < 1);
        boolean frontLeft = (Math.abs((driveSubsystem.robotDrive.frontLeftMotor.getCurrentPosition()
                * (Parameters.kInchesPerTick) - wheelTargets.get(1))) < 1);
        boolean backRight = (Math.abs((driveSubsystem.robotDrive.frontLeftMotor.getCurrentPosition()
                * (Parameters.kInchesPerTick) - wheelTargets.get(2))) < 1);
        boolean frontRight = (Math.abs((driveSubsystem.robotDrive.frontLeftMotor.getCurrentPosition()
                * (Parameters.kInchesPerTick) - wheelTargets.get(3))) < 1);
        boolean distanceReached = backLeft && frontLeft && backRight && frontRight;
        boolean timeoutReached = timer.seconds() >= timeout;

        return distanceReached || timeoutReached;
    }
    /*** Runs when command is finished ***/
    public void finish() {
        driveSubsystem.robotDrive.resetEncoders();
    }

}
