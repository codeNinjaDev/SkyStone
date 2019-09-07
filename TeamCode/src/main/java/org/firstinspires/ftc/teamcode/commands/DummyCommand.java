package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class DummyCommand implements Command {
    private DriveSubsystem drive;
    private double power, time;
    public DummyCommand(DriveSubsystem drive, double power, double time) {
        this.drive = drive;
        this.power = power;
        this.time = time;
    }

    @Override
    public void init() {
        //startTimer
    }

    @Override
    public void update(Telemetry tl) {
        drive.arcadeDrive(power, 0);
    }

    @Override
    public boolean isFinished() {
        return currentTIme - startTime > time;
    }

    @Override
    public void finish() {
        drive.arcadeDrive(0, 0);
    }
}
