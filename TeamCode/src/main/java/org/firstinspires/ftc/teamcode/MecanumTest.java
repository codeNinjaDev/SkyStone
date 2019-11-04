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

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.commands.DriveDistanceCommand;
import org.firstinspires.ftc.teamcode.commands.DrivePolarCommand;
import org.firstinspires.ftc.teamcode.commands.MecanumDriveCommand;
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

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="MecanumTest", group="Autos")  // @Autonomous(...) is the other common choice
//@Disabled
public class MecanumTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private CommandRunner goToFoundation;
    private CommandRunner pullFoundation;
    private CommandRunner strafeRight;
    private CommandRunner strafeDiagonally;
    private CommandRunner strafeDiagonallyOpposite;
    private CommandRunner strafeBack;
    private CommandRunner goToFoundationAgain;
    private CommandRunner park;



    //private CommandRunner drivePolar;

    VuSubsystem vu;

    DriveSubsystem driveController;


    @Override
    public void runOpMode() {
        vu = new VuSubsystem(hardwareMap, telemetry, true);
        driveController = new DriveSubsystem(hardwareMap, vu, gamepad1, telemetry);

        goToFoundation = new CommandRunner(this, new MecanumDriveCommand(driveController, 20, 90, 12, telemetry), telemetry);
        pullFoundation = new CommandRunner(this, new MecanumDriveCommand(driveController, -20, 90, 12, telemetry), telemetry);
        strafeRight = new CommandRunner(this, new MecanumDriveCommand(driveController, 60, 0, 12, telemetry), telemetry);
        strafeDiagonally = new CommandRunner(this, new MecanumDriveCommand(driveController, 60, 45, 12, telemetry), telemetry);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        goToFoundation.runCommand();
        pullFoundation.runCommand();
        strafeRight.runCommand();
        strafeDiagonally.runCommand();
        //drivePolar.runCommand();
        runtime.reset();


    }


}