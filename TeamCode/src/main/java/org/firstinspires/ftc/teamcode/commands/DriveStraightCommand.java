package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Parameters;
import org.firstinspires.ftc.teamcode.libs.PIDController;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/*** Drives a set distance in inches
 *
 *
 * ***/
public class DriveStraightCommand implements Command {
    private DriveSubsystem driveSubsystem;
    private PIDController positionPIDController, headingController;

    private ElapsedTime timer;
    double distance, timeout, speed;

    // Drive straight from robot's current or global perspecitive
    boolean globalHeading;

    public DriveStraightCommand(DriveSubsystem driveSubsystem, double distance, double speed
            , double timeout, boolean globalHeading) {
        this.driveSubsystem = driveSubsystem;
        positionPIDController = new PIDController(0.08, 0, .001, speed, 1);
        headingController = new PIDController(0.42, 0.000001, 0.001, 30, 3);
        this.globalHeading = globalHeading;

        this.timeout = timeout;
        timer = new ElapsedTime();

    }

    @Override
    public void init() {
        driveSubsystem.reset();
        positionPIDController.setSetpoint(distance, true);
        if(globalHeading)
            headingController.setSetpoint(0, true);
        else
            headingController.setSetpoint(driveSubsystem.getHeading(), true);

        timer.reset();

        timer.startTime();
    }


    @Override
    public void update(Telemetry tl) {

        double averageDistance = driveSubsystem.robotDrive.getAverageDistance();

        driveSubsystem.arcadeDrive(positionPIDController.run(averageDistance), -headingController.run(driveSubsystem.getHeading()), false);

    }

    @Override
    public boolean isFinished() {
        boolean timeoutReached = timer.seconds() >= timeout;
        boolean positionReached = (positionPIDController.onTarget() && headingController.onTarget());
        return timeoutReached || positionReached;
    }


    @Override
    public void finish() {
        driveSubsystem.stop();
    }
}



