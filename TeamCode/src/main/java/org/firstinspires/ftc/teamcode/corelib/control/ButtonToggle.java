package org.firstinspires.ftc.teamcode.corelib.control;

/**
 * Button Toggle - Detects button presses and toggles state on/off
 *
 * THE PROBLEM:
 * When you check if a button is pressed using gamepad.a, it returns true for
 * EVERY loop iteration while the button is held down. This means:
 *   - Hold button for 0.5 seconds = 50+ loop iterations of "button is pressed"
 *   - Your code would toggle on/off/on/off/on/off... rapidly!
 *
 * THE SOLUTION:
 * We only want to detect the MOMENT the button is pressed (the transition from
 * not pressed → pressed), not every frame it's held down.
 *
 * HOW IT WORKS:
 * 1. Remember the previous state of the button
 * 2. Each loop, check current state
 * 3. If previous = false and current = true → BUTTON PRESS DETECTED!
 * 4. Toggle the state (on → off or off → on)
 * 5. Save current state for next loop
 *
 * ANALOGY - Light switch:
 * Without toggle detection:
 *   - Hold switch down → lights flicker on/off/on/off rapidly
 *
 * With toggle detection:
 *   - Press switch → lights turn on, stay on even while holding
 *   - Release and press again → lights turn off
 *
 * EXAMPLE:
 * Frame 1: Button up → previousState = false
 * Frame 2: Button down → PRESS DETECTED! Toggle state, previousState = true
 * Frame 3: Button still down → No change (already true)
 * Frame 4: Button still down → No change (already true)
 * Frame 5: Button released → previousState = false
 * Frame 6: Button down again → PRESS DETECTED! Toggle state, previousState = true
 */
public class ButtonToggle {
    private boolean previousState = false;
    private boolean toggleState = false;

    /**
     * Update the toggle with current button state
     * @param currentButtonState True if button is currently pressed
     * @return True if button was just pressed this frame (rising edge)
     */
    public boolean update(boolean currentButtonState) {
        boolean pressed = false;

        // Detect rising edge (button was just pressed this frame)
        if (currentButtonState && !previousState) {
            pressed = true;
            toggleState = !toggleState; // Flip the toggle
        }

        previousState = currentButtonState;
        return pressed;
    }

    /**
     * Get the current toggle state
     * @return True if toggled on, false if toggled off
     */
    public boolean getState() {
        return toggleState;
    }

    /**
     * Set the toggle state manually
     * @param state The desired state
     */
    public void setState(boolean state) {
        this.toggleState = state;
    }

    /**
     * Reset the toggle to off
     */
    public void reset() {
        toggleState = false;
        previousState = false;
    }
}