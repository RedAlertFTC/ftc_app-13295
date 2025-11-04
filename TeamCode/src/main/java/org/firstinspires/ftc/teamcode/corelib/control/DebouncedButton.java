package org.firstinspires.ftc.teamcode.corelib.control;

public class DebouncedButton {
    private final IOnOffButton onOffButton;
    private boolean lastState;

    public DebouncedButton(IOnOffButton getButtonState) {
        this.onOffButton = getButtonState;

        this.lastState = onOffButton.isPressed();
    }

    public boolean getRise() {
        boolean currentButtonState = onOffButton.isPressed();

        if (currentButtonState && !lastState) {
            lastState = true;
            return true;
        }

        lastState = currentButtonState;
        return false;
    }

    public boolean getFall() {
        boolean currentButtonState = onOffButton.isPressed();

        if (!currentButtonState && lastState) {
            lastState = false;
            return true;
        }

        lastState = currentButtonState;
        return false;
    }
}
