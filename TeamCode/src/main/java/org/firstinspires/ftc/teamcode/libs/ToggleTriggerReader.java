package org.firstinspires.ftc.teamcode.libs;

public class ToggleTriggerReader extends TriggerReader {
    private boolean currToggleState;

    public ToggleTriggerReader(SuperGamepad gamepad, GamepadKeys.Trigger trigger) {
        super(gamepad, trigger);
        currToggleState = false;
    }

    public boolean getState() {
        if (wasJustReleased()) {
            currToggleState = !currToggleState;
        }
        return (currToggleState);
    }
}
