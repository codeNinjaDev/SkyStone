package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    public final double RIGHT_FOUNDATION_UP = 0.67;
    public final double RIGHT_FOUNDATION_DOWN = 0.15;
    public final double LEFT_FOUNDATION_UP = 0.35;
    public final double LEFT_FOUNDATION_DOWN = 0.68;

    public Servo leftFoundationServo, rightFoundationServo, zoneServo, capstoneServo;
    Telemetry tl;
    public EndgameSubsystem(SuperGamepad driverGamepad, HardwareMap hw, Telemetry tl) {
        this.hw = hw;
        this.driverGamepad = driverGamepad;
        this.tl = tl;
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
            moveFoundationDown();

        } else {
            moveFoundationUp();
        }
        tl.addData("Servo Pos", rightFoundationServo.getPosition());

        if(capstoneButton.isDown()) {
            capstoneServo.setPosition(0.47);
        } else {
            capstoneServo.setPosition(0.65);
        }
    }

    @Override
    public void stop() {

    }

    public void moveFoundationDown() {
        leftFoundationServo.setPosition(LEFT_FOUNDATION_DOWN);
        rightFoundationServo.setPosition(RIGHT_FOUNDATION_DOWN);
    }

    public void moveFoundationUp() {
        leftFoundationServo.setPosition(LEFT_FOUNDATION_UP);
        rightFoundationServo.setPosition(RIGHT_FOUNDATION_UP);
    }
}
