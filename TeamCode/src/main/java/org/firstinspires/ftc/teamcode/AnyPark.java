package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.libs.SuperGamepad;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AnyPark", group = "parking")  // @Autonomous(...) is the other common choice
public class AnyPark extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private SuperGamepad driverGamepad;
    private CommandRunner strafeFromWall;
    private CommandRunner park;

    DriveSubsystem driveController;

    @Override
    public void runOpMode() {

        driverGamepad = new SuperGamepad(gamepad1);
        driveController = new DriveSubsystem(hardwareMap, driverGamepad, telemetry);
        driveController.reset();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.update();
        sleep(200);

        //strafeFromWall = new CommandRunner(this, new MecanumDriveCommand(driveController, 22, 0, 15, 2, telemetry), telemetry);
        //strafeFromWall.runCommand();
        park = new CommandRunner(this, new MecanumDriveCommand(driveController, 4, 90, 30,10, telemetry), telemetry);
        park.runCommand();
    }

}
