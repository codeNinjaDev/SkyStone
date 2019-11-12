package org.firstinspires.ftc.teamcode.libs;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ButtonReader {
    private SuperGamepad gamepad;
    private Buttons button;
    /** Last state of the button **/
    private boolean lastState;
    /** Current state of the button **/
    private boolean currState;
    private Telemetry telemetry;
    /*** Description of Button ***/
    private String buttonName;
    /** Initializes controller variables
     * @param gamepad The controller joystick
     * @param button The controller button
    **/
    public ButtonReader(SuperGamepad gamepad, Buttons button) {
        this.gamepad = gamepad;
        this.button = button;
        currState = this.gamepad.getButton(button);
        lastState = currState;
    }

    public ButtonReader(SuperGamepad gamepad, Buttons button, String buttonName) {
        this.gamepad = gamepad;
        this.button = button;
        this.buttonName = buttonName + "_BUTTON";
        currState = this.gamepad.getButton(button);
        lastState = currState;
    }
    /** Reads button value **/
    public void readValue() {
        lastState = currState;
        currState = this.gamepad.getButton(button);
    }
    /** Checks if the button is down **/
    public boolean isDown() {
        return currState;
    }
    /** Checks if the button was just pressed **/
    public boolean wasJustPressed() {
        return (lastState == false && currState == true);
    }
    /** Checks if the button was just released **/
    public boolean wasJustReleased() {
        return (lastState == true && currState == false);
    }
    /** Checks if the button state has changed **/
    public boolean stateJustChanged() {
        return (lastState != currState);
    }
}
