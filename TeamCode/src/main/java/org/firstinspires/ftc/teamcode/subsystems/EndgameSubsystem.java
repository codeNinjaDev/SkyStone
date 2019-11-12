package org.firstinspires.ftc.teamcode.subsystems;

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
    private ToggleButtonReader foundationToggleButton;

    public Servo leftFoundationServo, rightFoundationServo, zoneServo;
    public EndgameSubsystem(SuperGamepad driverGamepad, HardwareMap hw) {
        this.hw = hw;
        this.driverGamepad = driverGamepad;

        leftFoundationServo = hw.servo.get("leftFServo");
        rightFoundationServo = hw.servo.get("rightFServo");

        zoneServo = hw.servo.get("zoneServo");
        goToZone = false;

        zoneScoreButton = new ButtonReader(driverGamepad, GamepadKeys.Button.BACK);
        foundationToggleButton = new ToggleButtonReader(driverGamepad, GamepadKeys.Button.Y);
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

        if(zoneScoreButton.isDown()) {
            goToZone = true;
        }
        if(goToZone) {
            zoneServo.setPosition(.5);
        } else {
            zoneServo.setPosition(0);
        }

        if(foundationToggleButton.getState()) {
            leftFoundationServo.setPosition(0.8);
            rightFoundationServo.setPosition(0.1);
        } else {
            leftFoundationServo.setPosition(0);
            rightFoundationServo.setPosition(0.95);
        }
    }

    @Override
    public void stop() {

    }
}
