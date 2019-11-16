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
    private boolean currentVisible, lastVisible;

    public TrackSkyStoneCommand(DriveSubsystem driveSubsystem, VuSubsystem vuSubsystem, double timeout, Telemetry tl) {
        this.driveSubsystem = driveSubsystem;
        this.vuSubsystem = vuSubsystem;
        this.timeout = timeout;
        timer = new ElapsedTime();
        lastVisible = false;
        currentVisible = false;
    }

    @Override
    public void init() {
        timer.reset();
        timer.startTime();
    }

    @Override
    public void update(Telemetry tl) {
        vuSubsystem.update();

        driveSubsystem.arcadeDrive(-0.085, 0, false);
    }

    @Override
    public boolean isFinished() {
        currentVisible = vuSubsystem.targetVisible;
        boolean distanceReached = currentVisible;
        lastVisible = currentVisible;

        return (timeout < timer.seconds()) || distanceReached;
    }

    @Override
    public void finish() {
        driveSubsystem.robotDrive.stopDriving();
    }
}
