package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.PIDController;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class TurnGyroCommand implements Command {
    PIDController gyroPID;
    DriveSubsystem driveSubsystem;
    private ElapsedTime timer;
    double angle, timeout, speed;

    public TurnGyroCommand(DriveSubsystem driveSubsystem, double angle, double timeout) {
        this.driveSubsystem = driveSubsystem;
        this.angle = angle;
        this.timeout = timeout;
        gyroPID = new PIDController(.012, 0.00, 0.0, 0.25, 8);
        timer = new ElapsedTime();
    }

    public TurnGyroCommand(DriveSubsystem driveSubsystem, double angle, double speed, double timeout) {
        this.driveSubsystem = driveSubsystem;
        this.angle = angle;
        this.timeout = timeout;
        gyroPID = new PIDController(.05, 0.00, 0.0001, speed, 10);
        timer = new ElapsedTime();
    }

    public void init() {
        driveSubsystem.reset();
        timer.reset();
        timer.startTime();
        gyroPID.setSetpoint(angle, true);
        gyroPID.setSetpoint(angle, true);
    }

    public void update(Telemetry tl) {
        tl.addData("Gyro: ", gyroPID.run(driveSubsystem.getHeading()));
        tl.update();
        driveSubsystem.arcadeDrive(0, -gyroPID.run(driveSubsystem.getHeading()), false);

    }

    public boolean isFinished() {
        boolean onTarget = gyroPID.onTarget();
        boolean timeoutReached = timer.seconds() > timeout;

        return onTarget || timeoutReached;
    }

    @Override
    public void finish() {
        driveSubsystem.stop();
    }
}
