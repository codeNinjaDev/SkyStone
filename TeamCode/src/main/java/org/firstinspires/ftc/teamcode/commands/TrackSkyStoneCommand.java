package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VuSubsystem;

public class TrackSkyStoneCommand implements Command {
    private DriveSubsystem driveSubsystem;
    private VuSubsystem vuSubsystem;
    private double timeout;

    public TrackSkyStoneCommand(DriveSubsystem driveSubsystem, VuSubsystem vuSubsystem, double timeout, Telemetry tl) {
        this.driveSubsystem = driveSubsystem;
        this.vuSubsystem = vuSubsystem;
        this.timeout = timeout;
    }

    @Override
    public void init() {
    }

    @Override
    public void update(Telemetry tl) {
        vuSubsystem.update();

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void finish() {

    }
}
