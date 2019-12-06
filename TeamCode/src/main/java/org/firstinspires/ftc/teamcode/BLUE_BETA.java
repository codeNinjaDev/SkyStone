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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.commands.MecanumDriveCommandSlow;
import org.firstinspires.ftc.teamcode.commands.MoveArmCommand;
import org.firstinspires.ftc.teamcode.commands.TrackSkyStoneCommand;
import org.firstinspires.ftc.teamcode.commands.TurnGyroCommand;
import org.firstinspires.ftc.teamcode.libs.SuperGamepad;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.EndgameSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SkystoneArm;
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
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="BLUE_25", group="25pt Autos")  // @Autonomous(...) is the other common choice
public class BLUE_BETA extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private SuperGamepad driverGamepad;
    private ArmSubsystem claws;

    private CommandRunner alignToSkystone;
    private CommandRunner goToSkystone;

    private CommandRunner strafeAwaySkystone1;

    private CommandRunner turnTowardsFoundation;
    private CommandRunner goToFoundation;
    private CommandRunner liftArm;
    private CommandRunner turnTowardsBuildingZone;

    private CommandRunner strafeIntoBuildingZone;
    private CommandRunner lowerArm;
    private CommandRunner pushPartner;
    private CommandRunner strafeFromWall;

    private CommandRunner park;
    VuSubsystem vu;

    DriveSubsystem driveController;
    SkystoneArm arms;
    EndgameSubsystem foundation;

    @Override
    public void runOpMode() {
        vu = new VuSubsystem(hardwareMap, telemetry, true);
        vu.init();

        driverGamepad = new SuperGamepad(gamepad1);
        driveController = new DriveSubsystem(hardwareMap, driverGamepad, telemetry);
        arms = new SkystoneArm(hardwareMap);
        claws = new ArmSubsystem(driverGamepad, hardwareMap);
        foundation = new EndgameSubsystem(driverGamepad, hardwareMap, telemetry);
        driveController.reset();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        for(int i = 0; i < 10; i++) {
            vu.update();
            sleep(10);
        }

        foundation.capstoneServo.setPosition(0.7);

        double firstSkystoneDistance = 24 + vu.horizontal_distance;
        alignToSkystone = new CommandRunner(this, new MecanumDriveCommand(driveController, vu.horizontal_distance - 4 - 12, 90, 24, 5, telemetry), telemetry);
        alignToSkystone.runCommand();
        goToSkystone = new CommandRunner(this, new MecanumDriveCommand(driveController, vu.distance + 2, 90, 24, 5, telemetry), telemetry);
        goToSkystone.runCommand();
        sleep(50);
        claws.closeClaw();

        strafeAwaySkystone1 = new CommandRunner(this, new MecanumDriveCommand(driveController, 12, 0, 18, 5, telemetry), telemetry);
        strafeAwaySkystone1.runCommand();


        goToFoundation = new CommandRunner(this, new MecanumDriveCommand(driveController, -(firstSkystoneDistance + 12), 90, 30, 5, telemetry), telemetry);
        goToFoundation.runCommand();

        turnTowardsFoundation = new CommandRunner(this, new TurnGyroCommand(driveController, 90, 1.5), telemetry);
        turnTowardsFoundation.runCommand();
        liftArm = new CommandRunner(this, new MoveArmCommand(claws, 311, 1.5), telemetry);
        liftArm.runCommand();
        foundation.moveFoundationDown();
        claws.openClaw();

        turnTowardsBuildingZone = new CommandRunner(this, new TurnGyroCommand(driveController, -90, 1.5), telemetry);
        turnTowardsBuildingZone.runCommand();

        strafeIntoBuildingZone = new CommandRunner(this, new MecanumDriveCommand(driveController, -20, 0, 30, 3, telemetry), telemetry);
        strafeIntoBuildingZone.runCommand();

        foundation.moveFoundationUp();

        sleep(50);
        lowerArm = new CommandRunner(this, new MoveArmCommand(claws, -200, 1.5), telemetry);
        lowerArm.runCommand();
        sleep(50);

        pushPartner = new CommandRunner(this, new MecanumDriveCommand(driveController, -36, 90, 30, 3, telemetry), telemetry);
        pushPartner.runCommand();

        strafeFromWall = new CommandRunner(this, new MecanumDriveCommand(driveController, -24, 0, 10, telemetry), telemetry);
        strafeFromWall.runCommand();

        park = new CommandRunner(this, new MecanumDriveCommand(driveController, -24, 90, 10, telemetry), telemetry);

        park.runCommand();
    }


}