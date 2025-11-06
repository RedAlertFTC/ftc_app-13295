package org.firstinspires.ftc.teamcode.corelib.control;

public class DebouncedGamepadButtons {
    private final DebouncedButton aButton;

    private final DebouncedButton bButton;

    private final DebouncedButton xButton;

    private final DebouncedButton yButton;

    private final DebouncedButton leftBumper;

    private final DebouncedButton rightBumper;

    private final DebouncedButton dpadUp;

    private final DebouncedButton dpadDown;

    private final DebouncedButton dpadLeft;

    private final DebouncedButton dpadRight;

    private final DebouncedButton leftStickButton;

    private final DebouncedButton rightStickButton;

    public DebouncedGamepadButtons(final DisasterGamePad gamepad) {
        aButton = new DebouncedButton(gamepad.getAButton());

        bButton = new DebouncedButton(gamepad.getBButton());

        xButton = new DebouncedButton(gamepad.getXButton());

        yButton = new DebouncedButton(gamepad.getYButton());

        leftBumper = new DebouncedButton(gamepad.getLeftBumper());

        rightBumper = new DebouncedButton(gamepad.getRightBumper());

        dpadUp = new DebouncedButton(gamepad.getDpadUp());

        dpadDown = new DebouncedButton(gamepad.getDpadDown());

        dpadLeft = new DebouncedButton(gamepad.getDpadLeft());

        dpadRight = new DebouncedButton(gamepad.getDpadRight());

        leftStickButton = new DebouncedButton(gamepad.getLeftStickButton());

        rightStickButton = new DebouncedButton(gamepad.getRightStickButton());
    }

    public DebouncedButton getaButton() {
        return aButton;
    }

    public DebouncedButton getbButton() {
        return bButton;
    }

    public DebouncedButton getxButton() {
        return xButton;
    }

    public DebouncedButton getyButton() {
        return yButton;
    }

    public DebouncedButton getLeftBumper() {
        return leftBumper;
    }

    public DebouncedButton getRightBumper() {
        return rightBumper;
    }

    public DebouncedButton getDpadUp() {
        return dpadUp;
    }

    public DebouncedButton getDpadDown() {
        return dpadDown;
    }

    public DebouncedButton getDpadLeft() {
        return dpadLeft;
    }

    public DebouncedButton getDpadRight() {
        return dpadRight;
    }

    public DebouncedButton getLeftStickButton() {
        return leftStickButton;
    }

    public DebouncedButton getRightStickButton() {
        return rightStickButton;
    }
}
