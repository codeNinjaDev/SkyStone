package org.firstinspires.ftc.teamcode.libs;

public class ToggleButtonReader extends ButtonReader {
    private boolean currToggleState;


    public ToggleButtonReader(SuperGamepad gamepad, GamepadKeys.Button button) {
        super(gamepad, button);

        currToggleState = false;

    }

    public boolean getState() {
        if (wasJustReleased()) {
            currToggleState = !currToggleState;
        }
        return (currToggleState);
    }
}
