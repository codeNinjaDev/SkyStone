package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.TFSubsystem;

public class TensorFlowTest extends OpMode {

    TFSubsystem tf;

    @Override
    public void init() {
        tf = new TFSubsystem(hardwareMap, telemetry, true);
        tf.init();
    }

    @Override
    public void loop() {
        tf.update();
    }

    @Override
    public void stop() {
        tf.stop();
    }
}
