package org.firstinspires.ftc.teamcode.libs;

import com.qualcomm.robotcore.hardware.Gamepad;


public class SuperGamepad {
    public Gamepad gamepad;

    public SuperGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public boolean getButton(GamepadKeys.Button button) {
        boolean buttonValue = false;
        switch (button) {
            case A:
                buttonValue = gamepad.a;
                break;
            case B:
                buttonValue = gamepad.b;
                break;
            case X:
                buttonValue = gamepad.x;
                break;
            case Y:
                buttonValue = gamepad.y;
                break;
            case LEFT_BUMPER:
                buttonValue = gamepad.left_bumper;
                break;
            case RIGHT_BUMPER:
                buttonValue = gamepad.right_bumper;
                break;
            case DPAD_UP:
                buttonValue = gamepad.dpad_up;
            case BACK:
                buttonValue = gamepad.back;
                break;
            default:
                buttonValue = false;
                break;
        }
        return buttonValue;
    }

    public double getTrigger(GamepadKeys.Trigger trigger) {
        double triggerValue = 0;
        switch (trigger) {
            case LEFT_TRIGGER:
                triggerValue = gamepad.left_trigger;
                break;
            case RIGHT_TRIGGER:
                triggerValue = gamepad.right_trigger;
                break;
            default:
                break;
        }
        return triggerValue;
    }

    public double getLeftY() {
        return -gamepad.left_stick_y;
    }

    public double getRightY() {
        return gamepad.right_stick_y;
    }

    public double getLeftX() {
        return gamepad.left_stick_x;
    }

    public double getRightX() {
        return gamepad.right_stick_x;
    }
}