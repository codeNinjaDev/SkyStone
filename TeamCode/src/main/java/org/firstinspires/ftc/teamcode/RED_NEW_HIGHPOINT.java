package org.firstinspires.ftc.teamcode;

/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

import com.arcrobotics.ftclib.vision.SkystoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.DrivePIDCommand;
import org.firstinspires.ftc.teamcode.commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.commands.MoveArmCommand;
import org.firstinspires.ftc.teamcode.commands.TurnGyroCommand;
import org.firstinspires.ftc.teamcode.libs.SuperGamepad;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.EndgameSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SkystoneArm;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VuSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.SkystoneArm;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="RED_NEW_HIGHPOINT", group = "29pt")  // @Autonomous(...) is the other common choice
public class RED_NEW_HIGHPOINT extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private SuperGamepad driverGamepad;
    private ArmSubsystem claws;
    private CommandRunner strafeCloser;
    private CommandRunner alignToSkystone;
    private CommandRunner goToSkystone;
    private CommandRunner getSkystone;
    private CommandRunner strafeAwaySkystone1;

    private CommandRunner turnTowardsFoundation;
    private CommandRunner moveTowardsFoundation;
    private CommandRunner goToFoundation;
    private CommandRunner liftArm;
    private CommandRunner turnTowardsBuildingZone;

    private CommandRunner moveFoundationIntoWall;
    private CommandRunner lowerArm;
    private CommandRunner strafeIntoWall;

    private CommandRunner pushPartner;
    private CommandRunner strafeFromWall;

    private CommandRunner park;
    VisionSubsystem visionSubsystem;

    DriveSubsystem driveController;
    SkystoneArm arms;
    EndgameSubsystem foundation;

    private final int FIRST_SKYSTONE = 1;
    private final int SECOND_SKYSTONE = 2;
    private final int THIRD_SKYSTONE = 3;

    private int SKYSTONE_POSITION;
    @Override
    public void runOpMode() {
        visionSubsystem = new VisionSubsystem(hardwareMap);

        driverGamepad = new SuperGamepad(gamepad1);
        driveController = new DriveSubsystem(hardwareMap, driverGamepad, telemetry);
        arms = new SkystoneArm(hardwareMap);
        claws = new ArmSubsystem(driverGamepad, hardwareMap);
        foundation = new EndgameSubsystem(driverGamepad, hardwareMap, telemetry);
        driveController.reset();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        SkystoneDetector.SkystonePosition position = visionSubsystem.getSkystonePosition();

        foundation.capstoneServo.setPosition(0.7);
        foundation.moveFoundationUp();
        strafeCloser = new CommandRunner(this, new MecanumDriveCommand(driveController, 16, 180, 15, 2, telemetry), telemetry);
        strafeCloser.runCommand();


        telemetry.update();
        sleep(200);

        switch (position) {
            case RIGHT_STONE:
                alignToSkystone = new CommandRunner(this, new DrivePIDCommand(driveController, -22, .4, 1.5), telemetry);
                break;
            case CENTER_STONE:
                alignToSkystone = new CommandRunner(this, new DrivePIDCommand(driveController, -13, .4, 1), telemetry);
                break;
            case LEFT_STONE:
                alignToSkystone = new CommandRunner(this, new DrivePIDCommand(driveController, -5.5, .4, 1), telemetry);
                break;
        }

        alignToSkystone.runCommand();

        goToSkystone = new CommandRunner(this, new MecanumDriveCommand(driveController, 15.25, 180, 34, 1.5, telemetry), telemetry);
        goToSkystone.runCommand();

        sleep(300);
        getSkystone = new CommandRunner(this, new DrivePIDCommand(driveController, 3, .4, 1), telemetry);
        getSkystone.runCommand();

        sleep(800);
        claws.closeClaw();
        sleep(500);

        strafeAwaySkystone1 = new CommandRunner(this, new MecanumDriveCommand(driveController, 13.5, 0, 34, 3, telemetry), telemetry);
        strafeAwaySkystone1.runCommand();

        sleep(100);

        switch (position) {
            case RIGHT_STONE:
                goToFoundation = new CommandRunner(this, new DrivePIDCommand(driveController, (92), 1, 3), telemetry);
                break;
            case CENTER_STONE:
                goToFoundation = new CommandRunner(this, new DrivePIDCommand(driveController, (86), 1, 3), telemetry);
                break;
            case LEFT_STONE:
                goToFoundation = new CommandRunner(this, new DrivePIDCommand(driveController, (75), 1, 2.5), telemetry);
                break;
        }

        goToFoundation.runCommand();
        sleep(600);

        liftArm = new CommandRunner(this, new MoveArmCommand(claws, 425, 1.75), telemetry);
        liftArm.runCommand();

        turnTowardsFoundation = new CommandRunner(this, new TurnGyroCommand(driveController, 90, .2,3), telemetry);
        turnTowardsFoundation.runCommand();

        moveTowardsFoundation = new CommandRunner(this, new MecanumDriveCommand(driveController, 18, 90, 16, .8, true,  telemetry), telemetry);
        moveTowardsFoundation.runCommand();
        foundation.moveFoundationDown();

        sleep(800);
        turnTowardsBuildingZone = new CommandRunner(this, new TurnGyroCommand(driveController, -30, .4, 4), telemetry);
        turnTowardsBuildingZone.runCommand();

        moveFoundationIntoWall = new CommandRunner(this, new MecanumDriveCommand(driveController, 50, 90, 30, 3, true, telemetry), telemetry);
        moveFoundationIntoWall.runCommand();

        claws.openClaw();
        foundation.moveFoundationUp();

        sleep(50);
        lowerArm = new CommandRunner(this, new MoveArmCommand(claws, -150, 0.5), telemetry);
        lowerArm.runCommand();
        sleep(50);

        CommandRunner alignRobot = new CommandRunner(this, new TurnGyroCommand(driveController, 0, .2, 2), telemetry);
        alignRobot.runCommand();
        CommandRunner giveSpace = new CommandRunner(this, new MecanumDriveCommand(driveController, 2, 180, 30,1, telemetry), telemetry);
        giveSpace.runCommand();
        park = new CommandRunner(this, new MecanumDriveCommand(driveController, 36, -90, 30,10, telemetry), telemetry);
        park.runCommand();
    }
}