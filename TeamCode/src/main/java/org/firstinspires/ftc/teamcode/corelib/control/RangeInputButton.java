package org.firstinspires.ftc.teamcode.corelib.control;

public class RangeInputButton implements IOnOffButton {
    private final IRangeInput input;

    private float threshold;

    public RangeInputButton(IRangeInput originalInput, float threshold) {
        this.input = originalInput;
        this.threshold = threshold;
    }

    @Override
    public boolean isPressed() {
        if (input.getPosition() >= threshold) {
            return true;
        }

        return false;
    }
}