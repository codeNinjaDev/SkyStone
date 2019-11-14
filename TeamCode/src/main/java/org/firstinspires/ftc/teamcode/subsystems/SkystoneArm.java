package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SkystoneArm implements Subsystem {
    Servo leftServo, rightServo;
    double DOWN_POSITION = 90;
    public SkystoneArm(HardwareMap hw) {
        leftServo = hw.servo.get("leftStoneServo");
        rightServo = hw.servo.get("rightStoneServo");

    }

    public void moveLeftArmUp() {
        leftServo.setPosition(0);
    }

    public void moveRightArmUp() {
        rightServo.setPosition(0);

    }

    public void moveLeftArmDown() {
        leftServo.setPosition(DOWN_POSITION);
    }

    public void moveRightArmDown() {
        rightServo.setPosition(DOWN_POSITION);
    }

    @Override
    public void init() {

    }

    @Override
    public void reset() {

    }

    @Override
    public void update() {
        leftServo.setPosition(0.1);
        rightServo.setPosition(0.65);
    }

    @Override
    public void stop() {

    }
}
