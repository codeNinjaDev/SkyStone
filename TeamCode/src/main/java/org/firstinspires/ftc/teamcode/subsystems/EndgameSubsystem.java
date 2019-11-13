package org.firstinspires.ftc.teamcode.subsystems;

import android.arch.core.util.Function;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.libs.ButtonReader;
import org.firstinspires.ftc.teamcode.libs.GamepadKeys;
import org.firstinspires.ftc.teamcode.libs.SuperGamepad;
import org.firstinspires.ftc.teamcode.libs.ToggleButtonReader;

public class EndgameSubsystem implements Subsystem {
    SuperGamepad driverGamepad;
    HardwareMap hw;
    boolean foundationToggle = true;
    boolean goToZone;

    private ButtonReader zoneScoreButton;
    private ButtonReader capstoneButton;
    private ToggleButtonReader foundationToggleButton;

    public Servo leftFoundationServo, rightFoundationServo, zoneServo, capstoneServo;
    public EndgameSubsystem(SuperGamepad driverGamepad, HardwareMap hw) {
        this.hw = hw;
        this.driverGamepad = driverGamepad;

        leftFoundationServo = hw.servo.get("leftFServo");
        rightFoundationServo = hw.servo.get("rightFServo");
        capstoneServo = hw.servo.get("capstoneServo");

        zoneServo = hw.servo.get("zoneServo");
        goToZone = false;

        zoneScoreButton = new ButtonReader(driverGamepad, GamepadKeys.Button.BACK);
        foundationToggleButton = new ToggleButtonReader(driverGamepad, GamepadKeys.Button.Y);
        capstoneButton = new ButtonReader(driverGamepad, GamepadKeys.Button.B);
    }

    @Override
    public void init() {

    }

    @Override
    public void reset() {

    }

    @Override
    public void update() {
        zoneScoreButton.readValue();
        foundationToggleButton.readValue();
        capstoneButton.readValue();

        if(zoneScoreButton.isDown()) {
            goToZone = true;
        }
        if(goToZone) {
            zoneServo.setPosition(.5);
        } else {
            zoneServo.setPosition(0);
        }

        if(foundationToggleButton.getState()) {
            leftFoundationServo.setPosition(0.85);
            rightFoundationServo.setPosition(0.05);
        } else {
            leftFoundationServo.setPosition(0);
            rightFoundationServo.setPosition(0.98);
        }

        if(capstoneButton.isDown()) {
            capstoneServo.setPosition(0.3);
        } else {
            capstoneServo.setPosition(0.6);
        }
    }

    @Override
    public void stop() {

    }
}
