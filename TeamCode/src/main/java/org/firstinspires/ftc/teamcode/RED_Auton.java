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
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.commands.MecanumDriveCommandSlow;
import org.firstinspires.ftc.teamcode.commands.TrackSkyStoneCommand;
import org.firstinspires.ftc.teamcode.commands.TurnGyroCommand;
import org.firstinspires.ftc.teamcode.libs.SuperGamepad;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VuSubsystem;

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
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="RED_Auton", group="Autos")  // @Autonomous(...) is the other common choice
@Disabled
public class RED_Auton extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private SuperGamepad driverGamepad;
    private CommandRunner goToFoundation;
    private CommandRunner pullFoundation;
    private CommandRunner strafeOut;
    private CommandRunner getOffTheWall;
    private CommandRunner turnTowardBridge;
    private CommandRunner goToBlocks;
    private CommandRunner goToFirstSkyStone;
    private CommandRunner lineUpBetter;
    private CommandRunner getFirstSkystone;
    private CommandRunner strafeAwayWithSkystone1;
    private CommandRunner driveBackToBase;
    private CommandRunner goToSecondSkystone;
    private CommandRunner getSecondSkystone;
    private CommandRunner strafeAwayWithSkystone2;

    private CommandRunner park;

    //private CommandRunner drivePolar;

    VuSubsystem vu;

    DriveSubsystem driveController;
    //SkystoneArm arms;
    Servo leftFoundationServo, rightFoundationServo;

    @Override
    public void runOpMode() {
        vu = new VuSubsystem(hardwareMap, telemetry, true);
        vu.init();

        leftFoundationServo = hardwareMap.servo.get("leftFServo");
        rightFoundationServo = hardwareMap.servo.get("rightFServo");
        driverGamepad = new SuperGamepad(gamepad1);
        driveController = new DriveSubsystem(hardwareMap, driverGamepad, telemetry);
        //arms = new SkystoneArm(hardwareMap);
        driveController.reset();

        goToFoundation = new CommandRunner(this, new MecanumDriveCommandSlow(driveController, -28, 90, 5, telemetry), telemetry);
        pullFoundation = new CommandRunner(this, new MecanumDriveCommand(driveController, 26.5, 90, 10, telemetry), telemetry);
        strafeOut = new CommandRunner(this, new MecanumDriveCommand(driveController, -35, 0, 10, telemetry), telemetry);
        getOffTheWall = new CommandRunner(this, new MecanumDriveCommand(driveController, -20, 90, 10, telemetry), telemetry);

        turnTowardBridge = new CommandRunner(this, new TurnGyroCommand(driveController, -91, 5), telemetry);
        goToBlocks = new CommandRunner(this, new MecanumDriveCommand(driveController, 35, 90, 10, telemetry), telemetry);

        goToFirstSkyStone = new CommandRunner(this, new TrackSkyStoneCommand(driveController, vu, 20, telemetry), telemetry);
        getFirstSkystone = new CommandRunner(this, new MecanumDriveCommand(driveController, -12, 0, 10, telemetry), telemetry);
        strafeAwayWithSkystone1 = new CommandRunner(this, new MecanumDriveCommand(driveController, 12, 0, 10, telemetry), telemetry);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        goToFoundation.runCommand();
        leftFoundationServo.setPosition(0.8);
        rightFoundationServo.setPosition(0.1);
        sleep(1500);
        pullFoundation.runCommand();
        leftFoundationServo.setPosition(0.1);
        rightFoundationServo.setPosition(0.8);
        sleep(250);
        strafeOut.runCommand();
        sleep(50);
        getOffTheWall.runCommand();
        sleep(50);
        turnTowardBridge.runCommand();
        sleep(50);

        telemetry.addData("Gyro: ", driveController.getHeading());
        telemetry.update();
        goToBlocks.runCommand();
        goToFirstSkyStone.runCommand();
        telemetry.addData("TargetVisibility, ", vu.targetVisible);
        telemetry.addData("horiz_distance, ", vu.horizontal_distance);
        telemetry.update();
        double firstSkystoneDistance = driveController.robotDrive.getAverageDistance() + 33;
        firstSkystoneDistance +=  vu.horizontal_distance;
        lineUpBetter = new CommandRunner(this, new MecanumDriveCommand(driveController, vu.horizontal_distance, 90, 0.3,4, telemetry), telemetry);
        sleep(50);
        lineUpBetter.runCommand();
        sleep(50);
        getFirstSkystone.runCommand();
        //arms.moveLeftArmDown();
        sleep(5000);
        strafeAwayWithSkystone1.runCommand();
        //arms.moveLeftArmUp();
        sleep(500);
        driveBackToBase = new CommandRunner(this, new MecanumDriveCommand(driveController, -firstSkystoneDistance, 90, 4, telemetry), telemetry);
        driveBackToBase.runCommand();
        goToSecondSkystone = new CommandRunner(this, new MecanumDriveCommand(driveController, firstSkystoneDistance + 16, 90, 4, telemetry), telemetry);
        goToSecondSkystone.runCommand();
        getSecondSkystone = new CommandRunner(this, new MecanumDriveCommand(driveController, -12, 0, 4, telemetry), telemetry);
        getSecondSkystone.runCommand();
        //arms.moveLeftArmDown();
        sleep(500);
        strafeAwayWithSkystone2 = new CommandRunner(this, new MecanumDriveCommand(driveController, 12, 0, 4, telemetry), telemetry);
        strafeAwayWithSkystone2.runCommand();
        park = new CommandRunner(this, new MecanumDriveCommand(driveController, -(firstSkystoneDistance + 16), 90, 4, telemetry), telemetry);
        park.runCommand();
        //arms.moveLeftArmUp();
        //       runtime.reset();

    }
}