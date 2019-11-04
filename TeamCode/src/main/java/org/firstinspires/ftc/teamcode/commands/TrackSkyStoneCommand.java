package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Parameters;
import org.firstinspires.ftc.teamcode.libs.PIDController;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VuSubsystem;

import java.util.ArrayList;

public class TrackSkyStoneCommand implements Command {
    private DriveSubsystem driveSubsystem;
    private VuSubsystem vuSubsystem;
    private boolean followTarget;

    private double y_distance, x_distance;
    private ArrayList<Double> wheelTargets;
    private ElapsedTime timer;
    private double timeout;
    double backLeftFrontRightDistance, backRightFrontLeftDistance;
    ArrayList<PIDController> wheelControllers;

    public TrackSkyStoneCommand(DriveSubsystem driveSubsystem, VuSubsystem vuSubsystem, boolean followTarget, double timeout, Telemetry tl) {
        this.driveSubsystem = driveSubsystem;
        this.vuSubsystem = vuSubsystem;
        this.timeout = timeout;
        this.followTarget = followTarget;
        wheelControllers = new ArrayList<PIDController>();
        wheelControllers.add(new PIDController(0.001, 0.0, 0.0, 0.6, 5));
        wheelControllers.add(new PIDController(0.001, 0.0, 0.0, 0.6, 5));
        wheelControllers.add(new PIDController(0.001, 0.0, 0.0, 0.6, 5));
        wheelControllers.add(new PIDController(0.001, 0.0, 0.0, 0.6, 5));


    }

    @Override
    public void init() {

    }

    @Override
    public void update(Telemetry tl) {
        vuSubsystem.update();

        if(vuSubsystem.targetVisible) {
            tl.addData("Target Distance: ", vuSubsystem.getDistance());
            tl.addData("Target Yaw: ", vuSubsystem.getYaw());

            backLeftFrontRightDistance = vuSubsystem.getDistance() + vuSubsystem.horizontal_distance;
            backRightFrontLeftDistance = vuSubsystem.getDistance() - vuSubsystem.horizontal_distance;

            for(int i = 0; i < wheelControllers.size(); i++) {
                if(i == 0 && i == 3) {
                    wheelControllers.get(i).setSetpoint(backLeftFrontRightDistance);

                } else {
                    wheelControllers.get(i).setSetpoint(backRightFrontLeftDistance);
                }
            }
            driveSubsystem.robotDrive.backLeftMotor.setPower(wheelControllers.get(0).run(driveSubsystem.robotDrive.backLeftMotor.getCurrentPosition() * Parameters.kInchesPerTick));
            driveSubsystem.robotDrive.frontLeftMotor.setPower(wheelControllers.get(1).run(driveSubsystem.robotDrive.frontLeftMotor.getCurrentPosition() * Parameters.kInchesPerTick));
            driveSubsystem.robotDrive.backRightMotor.setPower(wheelControllers.get(2).run(driveSubsystem.robotDrive.backRightMotor.getCurrentPosition() * Parameters.kInchesPerTick));
            driveSubsystem.robotDrive.frontRightMotor.setPower(wheelControllers.get(3).run(driveSubsystem.robotDrive.frontRightMotor.getCurrentPosition() * Parameters.kInchesPerTick));

        }
    }

    @Override
    public boolean isFinished() {
        boolean distanceReached = wheelControllers.get(0).onTarget() && wheelControllers.get(1).onTarget()
                && wheelControllers.get(2).onTarget() && wheelControllers.get(3).onTarget();
        return timeout < timer.seconds() || distanceReached;
    }

    @Override
    public void finish() {
        driveSubsystem.robotDrive.stopDriving();
    }
}
