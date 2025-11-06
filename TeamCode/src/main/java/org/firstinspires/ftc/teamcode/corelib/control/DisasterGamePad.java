package org.firstinspires.ftc.teamcode.corelib.control;

import com.qualcomm.robotcore.hardware.Gamepad;

public class DisasterGamePad {
    private final Gamepad gamepad;

    public DisasterGamePad(final Gamepad originalGamepad) {
        gamepad = originalGamepad;
    }

    public IOnOffButton getAButton(){
        return new IOnOffButton() {

            @Override
            public boolean isPressed() {
                return gamepad.a;
            }
        };
    }

    public IOnOffButton getBButton(){
        return new IOnOffButton() {

            @Override
            public boolean isPressed() {
                return gamepad.b;
            }
        };
    }

    public IOnOffButton getXButton() {
        return new IOnOffButton() {

            @Override
            public boolean isPressed() {
                return gamepad.x;
            }
        };
    }

    public IOnOffButton getYButton(){
        return new IOnOffButton() {

            @Override
            public boolean isPressed() {
                return gamepad.y;
            }
        };
    }

    public IOnOffButton getLeftBumper(){
        return new IOnOffButton() {

            @Override
            public boolean isPressed() {
                return gamepad.left_bumper;
            }
        };
    }

    public IOnOffButton getRightBumper(){
        return new IOnOffButton() {

            @Override
            public boolean isPressed() {
                return gamepad.right_bumper;
            }
        };
    }

    public IOnOffButton getDpadUp(){
        return new IOnOffButton() {

            @Override
            public boolean isPressed() {
                return gamepad.dpad_up;
            }
        };
    }

    public IOnOffButton getDpadDown(){
        return new IOnOffButton() {

            @Override
            public boolean isPressed() {
                return gamepad.dpad_down;
            }
        };
    }

    public IOnOffButton getDpadLeft(){
        return new IOnOffButton() {

            @Override
            public boolean isPressed() {
                return gamepad.dpad_left;
            }
        };
    }

    public IOnOffButton getDpadRight(){
        return new IOnOffButton() {

            @Override
            public boolean isPressed() {
                return gamepad.dpad_right;
            }
        };
    }

    public IOnOffButton getLeftStickButton(){
        return new IOnOffButton() {

            @Override
            public boolean isPressed() {
                return gamepad.left_stick_button;
            }
        };
    }

    public IOnOffButton getRightStickButton(){
        return new IOnOffButton() {

            @Override
            public boolean isPressed() {
                return gamepad.right_stick_button;
            }
        };
    }

    public IRangeInput getLeftStickY(){
        return new IRangeInput() {
            @Override
            public float getPosition() {
                return gamepad.left_stick_y;
            }

            @Override
            public float getMaxPosition() {
                return 1.0f;
            }

            @Override
            public float getMinPosition() {
                return -1.0f;
            }
        };
    }

    public IRangeInput getLeftStickX(){
        return new IRangeInput() {
            @Override
            public float getPosition() {
                return gamepad.left_stick_x;
            }

            @Override
            public float getMaxPosition() {
                return 1.0f;
            }

            @Override
            public float getMinPosition() {
                return -1.0f;
            }
        };
    }

    public IRangeInput getRightStickY(){
        return new IRangeInput() {
            @Override
            public float getPosition() {
                return gamepad.right_stick_y;
            }

            @Override
            public float getMaxPosition() {
                return 1.0f;
            }

            @Override
            public float getMinPosition() {
                return -1.0f;
            }
        };
    }

    public IRangeInput getRightStickX(){
        return new IRangeInput() {
            @Override
            public float getPosition() {
                return gamepad.right_stick_x;
            }

            @Override
            public float getMaxPosition() {
                return 1.0f;
            }

            @Override
            public float getMinPosition() {
                return -1.0f;
            }
        };
    }

    public IRangeInput getLeftTrigger(){
        return new IRangeInput() {
            @Override
            public float getPosition() {
                return gamepad.left_trigger;
            }

            @Override
            public float getMaxPosition() {
                return 1.0f;
            }

            @Override
            public float getMinPosition() {
                return 0.0f;
            }
        };
    }

    public IRangeInput getRightTrigger(){
        return new IRangeInput() {
            @Override
            public float getPosition() {
                return gamepad.right_trigger;
            }

            @Override
            public float getMaxPosition() {
                return 1.0f;
            }

            @Override
            public float getMinPosition() {
                return 0.0f;
            }
        };
    }
}
