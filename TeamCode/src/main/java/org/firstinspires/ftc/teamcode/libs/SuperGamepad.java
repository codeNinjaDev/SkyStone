package org.firstinspires.ftc.teamcode.libs;

import com.qualcomm.robotcore.hardware.Gamepad;



public class SuperGamepad extends Gamepad {
    Gamepad gamepad;
    public SuperGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public boolean getButton(Buttons button) {
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
            case BACK:
                buttonValue = gamepad.back;
                break;
            default:
                buttonValue = false;
                break;
        }
        return buttonValue;
    }
}
