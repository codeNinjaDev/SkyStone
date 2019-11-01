package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.TFSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VuSubsystem;

@TeleOp(name="Vuforia", group="Iterative Opmode")  // @Autonomous(...) is the other common choic
public class VuforiaTest extends OpMode {

    VuSubsystem vu;

    @Override
    public void init() {
        vu = new VuSubsystem(hardwareMap, telemetry, true);
        vu.init();
    }

    @Override
    public void loop() {
        vu.update();
    }

    @Override
    public void stop() {
        vu.stop();
    }
}
