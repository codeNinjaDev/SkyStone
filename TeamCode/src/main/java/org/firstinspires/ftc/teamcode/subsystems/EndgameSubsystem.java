package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class EndgameSubsystem implements Subsystem {
    Gamepad driverGamepad;
    HardwareMap hw;
    boolean foundationToggle = true;
    boolean goToZone;
    public Servo leftFoundationServo, rightFoundationServo, zoneServo;
    public EndgameSubsystem(Gamepad driverGamepad, HardwareMap hw) {
        this.hw = hw;
        this.driverGamepad = driverGamepad;

        leftFoundationServo = hw.servo.get("leftFServo");
        rightFoundationServo = hw.servo.get("rightFServo");

        zoneServo = hw.servo.get("zoneServo");
        goToZone = false;
    }

    @Override
    public void init() {

    }

    @Override
    public void reset() {

    }

    @Override
    public void update() {

        if(driverGamepad.back) {
            goToZone = true;
        }
        if(goToZone) {
            zoneServo.setPosition(.5);
        } else {
            zoneServo.setPosition(0);
        }

        if(driverGamepad.y) {
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
