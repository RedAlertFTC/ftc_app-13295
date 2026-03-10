package org.firstinspires.ftc.teamcode.season2025.Components;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Coordinates the BallLauncher, Turntable, and BallSpooner to shoot multiple balls in sequence.
 *
 * Usage:
 *   ShooterSequence seq = new ShooterSequence(launcher, turntable, spooner, telemetry);
 *   seq.shootAllRapidlyAsync(); // shoots 3 balls using defaults without blocking main thread
 *
 * The class provides both a blocking method (shootAllRapidly) and an async wrapper that starts a
 * background thread. The blocking method now checks a stopRequested flag so it can be cancelled.
 */
public class ShooterSequence {

    private final BallLauncher launcher;
    private final Turntable turntable;
    private final BallSpooner spooner;
    private final Telemetry telemetry;

    // Defaults
    private final long DEFAULT_SPINUP_MS = 800; // ms to wait for launcher to reach speed
    private final long DEFAULT_BETWEEN_SHOTS_MS = 300; // small pause between shots/advances
    private final long DEFAULT_SPOONER_TIMEOUT_MS = 1400; // timeout per spooner fire
    private final long DEFAULT_TURNTABLE_SETTLE_MS = 250; // wait after moving turntable

    // runtime state for async operation
    private volatile boolean running = false;
    private volatile boolean stopRequested = false;

    // Launcher readiness tolerances
    private final double LAUNCHER_TPS_TOLERANCE_FRAC = 0.02; // 5% tolerance for velocity mode
    private final double LAUNCHER_POWER_TOLERANCE = 0.03; // absolute power tolerance for open-loop power mode
    private final long LAUNCHER_READY_TIMEOUT_MS = 1500; // how long to wait for launcher to settle before giving up

    public ShooterSequence(BallLauncher launcher, Turntable turntable, BallSpooner spooner, Telemetry telemetry) {
        this.launcher = launcher;
        this.turntable = turntable;
        this.spooner = spooner;
        this.telemetry = telemetry;
    }

    /** Convenience: shoot 3 balls using defaults (blocking). */
    public void shootAllRapidly() {
        shootAllRapidly(3, DEFAULT_SPINUP_MS, DEFAULT_BETWEEN_SHOTS_MS, DEFAULT_SPOONER_TIMEOUT_MS);
    }

    /**
     * Shoot balls in a specific turntable slot order. Each entry in `order` is a 1-based slot index.
     * The method will home the turntable to slot 1 first (safety), spin up the launcher, then move to each
     * requested slot and fire the spooner once.
     */
    public void shootInOrder(int[] order) {
        shootInOrder(order, DEFAULT_SPINUP_MS, DEFAULT_BETWEEN_SHOTS_MS, DEFAULT_SPOONER_TIMEOUT_MS);
    }

    /** Async wrapper for shootInOrder. */
    public void shootInOrderAsync(final int[] order) {
        if (running) return;
        stopRequested = false;
        Thread t = new Thread(() -> {
            running = true;
            try { shootInOrder(order); } finally { running = false; }
        }, "ShooterSequence-Order-Thread");
        t.setDaemon(true);
        t.start();
    }

    public void shootInOrder(int[] order, long spinUpMs, long betweenShotsMs, long spoonerTimeoutMs) {
        if (order == null || order.length == 0) return;
        if (launcher == null || turntable == null || spooner == null) {
            if (telemetry != null) telemetry.addData("ShooterSequence", "missing components");
            return;
        }

        stopRequested = false;

        // Home turntable to slot 1 first (same safety as other method)
        try {
            if (telemetry != null) telemetry.addData("ShooterSequence", "Homing turntable to slot 1 (pre-order)");
            turntable.moveToIndex(1);
            try { turntable.updateCurrentSlot(); } catch (Exception ignored) {}
            long settleStart = System.currentTimeMillis();
            final long FORCE_SETTLE_MS = Math.max(DEFAULT_TURNTABLE_SETTLE_MS, 500);
            while (!stopRequested && (System.currentTimeMillis() - settleStart) < FORCE_SETTLE_MS) {
                try { Thread.sleep(10); } catch (InterruptedException ignored) {}
            }
        } catch (Exception e) {
            if (telemetry != null) telemetry.addData("ShooterSequence", "turntable home failed: %s", e.getMessage());
        }

        // Spin up launcher
        long t0 = System.currentTimeMillis();
        while (!stopRequested && System.currentTimeMillis() - t0 < spinUpMs) {
            if (telemetry != null) telemetry.addData("ShooterSequence", "spinning up... %dms", System.currentTimeMillis() - t0);
            try { Thread.sleep(20); } catch (InterruptedException ignored) {}
        }

        if (stopRequested) return;

        int shotsFired = 0;
        for (int idx = 0; idx < order.length && !stopRequested; idx++) {
            int slot = order[idx];
            if (telemetry != null) telemetry.addData("ShooterSequence", "Moving to slot %d for shot %d/%d", slot, idx+1, order.length);
            try {
                turntable.moveToIndex(slot);
                try { turntable.updateCurrentSlot(); } catch (Exception ignored) {}
            } catch (Exception ignored) {}

            long settleStart = System.currentTimeMillis();
            while (!stopRequested && System.currentTimeMillis() - settleStart < DEFAULT_TURNTABLE_SETTLE_MS) {
                try { Thread.sleep(10); } catch (InterruptedException ignored) {}
            }

            // Fire spooner
            // Ensure launcher is ready before firing. If not ready within timeout, abort remaining shots.
            boolean ready = waitForLauncherReady(LAUNCHER_READY_TIMEOUT_MS);
            if (!ready) {
                if (telemetry != null) telemetry.addData("ShooterSequence", "Launcher not ready for shot %d; aborting remaining shots", idx+1);
                break;
            }
            spooner.fire();

            long start = System.currentTimeMillis();
            boolean spoonerDone = false;
            while (!stopRequested && System.currentTimeMillis() - start < spoonerTimeoutMs) {
                try { spooner.updateSpoonerState(); } catch (Exception ignored) {}
                if (!spooner.isBusy()) { spoonerDone = true; break; }
                try { Thread.sleep(10); } catch (InterruptedException ignored) {}
            }
            if (spoonerDone) shotsFired++;

            // small pause
            long betweenStart = System.currentTimeMillis();
            while (!stopRequested && System.currentTimeMillis() - betweenStart < betweenShotsMs) {
                try { Thread.sleep(10); } catch (InterruptedException ignored) {}
            }
        }

        try { launcher.turnOffLauncher(); } catch (Exception ignored) {}
        if (telemetry != null) telemetry.addData("ShooterSequence", "finished ordered shots: shotsFired=%d", shotsFired);
    }

    /** Convenience: start async shoot of 3 balls using defaults. Returns immediately. */
    public void shootAllRapidlyAsync() {
        shootAllRapidlyAsync(3, DEFAULT_SPINUP_MS, DEFAULT_BETWEEN_SHOTS_MS, DEFAULT_SPOONER_TIMEOUT_MS);
    }

    /** Start an async shoot. If already running this call is ignored. */
    public void shootAllRapidlyAsync(final int count, final long spinUpMs, final long betweenShotsMs, final long spoonerTimeoutMs) {
        if (running) return;
        stopRequested = false;
        Thread t = new Thread(() -> {
            running = true;
            try {
                shootAllRapidly(count, spinUpMs, betweenShotsMs, spoonerTimeoutMs);
            } finally {
                running = false;
            }
        }, "ShooterSequence-Thread");
        t.setDaemon(true);
        t.start();
    }

    /** Return true if an async shoot is in progress. */
    public boolean isRunning() {
        return running;
    }

    /** Request that a running shoot cancel as soon as possible. */
    public void requestStop() {
        stopRequested = true;
    }

    /**
     * Shoot `count` balls in sequence.
     * This method blocks while firing unless stopRequested is set.
     *
     * @param count how many balls to attempt to fire (use 0 for none)
     * @param spinUpMs time to wait after starting launcher before first shot
     * @param betweenShotsMs delay between shots / turntable advances
     * @param spoonerTimeoutMs maximum time to wait for spooner to finish a single fire
     */
    public void shootAllRapidly(int count, long spinUpMs, long betweenShotsMs, long spoonerTimeoutMs) {
        if (count <= 0) return;
        if (launcher == null || turntable == null || spooner == null) {
            if (telemetry != null) telemetry.addData("ShooterSequence", "missing components");
            return;
        }

        stopRequested = false;

        // Ensure turntable returns to slot 1 before starting the shooting sequence.
        try {
            if (telemetry != null) { telemetry.addData("ShooterSequence", "Homing turntable to slot 1"); }

            // Force move to slot 1 and update servo command
            turntable.moveToIndex(1);
            try { turntable.updateCurrentSlot(); } catch (Exception ignored) {}

            // Wait a fixed settle period to allow the servo to physically move (still cancelable)
            long settleStart = System.currentTimeMillis();
            final long FORCE_SETTLE_MS = Math.max(DEFAULT_TURNTABLE_SETTLE_MS, 500);
            while (!stopRequested && (System.currentTimeMillis() - settleStart) < FORCE_SETTLE_MS) {
                try { Thread.sleep(10); } catch (InterruptedException ignored) {}
            }

            if (telemetry != null) {
                try { telemetry.addData("ShooterSequence", "Turntable homing done; slot=%d busy=%b", turntable.currentSlot(), turntable.isBusy()); } catch (Exception ignored) {}

            }
        } catch (Exception e) {
            if (telemetry != null) telemetry.addData("ShooterSequence", "turntable home failed: %s", e.getMessage());
        }

        try {
            // Spin up the launcher to a preset velocity if available (use preset two as a reasonable default).
            // Caller may prefer to set specific velocity before calling.
            //launcher.setLaunchPresetTwo();
        } catch (Exception e) {
            // ignore
        }

        // Wait for spinner to come up to speed (check stopRequested so we can abort)
        long t0 = System.currentTimeMillis();
        while (!stopRequested && System.currentTimeMillis() - t0 < spinUpMs) {
            if (telemetry != null) telemetry.addData("ShooterSequence", "spinning up... %dms", System.currentTimeMillis() - t0);

            try { Thread.sleep(20); } catch (InterruptedException ignored) { }
        }

        if (stopRequested) {
            if (telemetry != null) { telemetry.addData("ShooterSequence","aborted before firing"); }
            return;
        }

        int shotsFired = 0;

        for (int i = 0; i < count && !stopRequested; i++) {
            // Fire the spooner once
            if (telemetry != null) telemetry.addData("ShooterSequence", "Firing ball %d/%d", i+1, count);

            // Ensure launcher is at commanded setpoint before firing this shot
            boolean ready = waitForLauncherReady(LAUNCHER_READY_TIMEOUT_MS);
            if (!ready) {
                if (telemetry != null) telemetry.addData("ShooterSequence", "Launcher not ready for shot %d; aborting", i+1);
                break;
            }
            spooner.fire();
            long start = System.currentTimeMillis();
            boolean spoonerDone = false;
            while (!stopRequested && System.currentTimeMillis() - start < spoonerTimeoutMs) {
                // caller should be calling spooner.updateSpoonerState() each loop; but to be safe, call it here
                try { spooner.updateSpoonerState(); } catch (Exception ignored) {}

                if (!spooner.isBusy()) {
                    spoonerDone = true;
                    break;
                }
                try { Thread.sleep(10); } catch (InterruptedException ignored) {}
            }

            if (stopRequested) break;

            if (!spoonerDone) {
                if (telemetry != null) telemetry.addData("ShooterSequence", "Spooner timeout on shot %d", i+1);

            } else {
                shotsFired++;
            }

            // small pause before moving the turntable so the ball clears
            long betweenStart = System.currentTimeMillis();
            while (!stopRequested && System.currentTimeMillis() - betweenStart < betweenShotsMs) {
                try { Thread.sleep(10); } catch (InterruptedException ignored) {}
            }

            if (stopRequested) break;

            // Advance the turntable to next slot so next ball is over spooner
            // increaseIndex() will do nothing if spooner is busy internally
            turntable.increaseIndex();
            // Give time for turntable to move and settle (also check stopRequested while sleeping)
            long settleStart = System.currentTimeMillis();
            while (!stopRequested && System.currentTimeMillis() - settleStart < DEFAULT_TURNTABLE_SETTLE_MS) {
                try { Thread.sleep(10); } catch (InterruptedException ignored) {}
            }

            // ensure turntable updates its servo position
            try { turntable.updateCurrentSlot(); } catch (Exception ignored) {}
        }

        // optional: stop launcher
        try {
            launcher.turnOffLauncher();
        } catch (Exception ignored) {}

        if (telemetry != null) {
            telemetry.addData("ShooterSequence", "finished: shotsFired=%d", shotsFired);

        }
    }

    /**
     * Wait for the launcher to reach its commanded setpoint (either TPS or power) within tolerances.
     * Returns true if the launcher is within tolerance within the timeout, false otherwise.
     */
    private boolean waitForLauncherReady(long timeoutMs) {
        if (launcher == null) return false;
        long start = System.currentTimeMillis();
        double desiredTPS = 0.0;
        double desiredPower = 0.0;
        try {
            desiredTPS = launcher.getCurrentTPS();
            desiredPower = launcher.getCurrentPower();
        } catch (Exception ignored) {}

        while (!stopRequested && System.currentTimeMillis() - start < timeoutMs) {
            try {
                double leftVel = launcher.getLeftVelocity();
                double rightVel = launcher.getRightVelocity();
                double avgVel = (Math.abs(leftVel) + Math.abs(rightVel)) / 2.0;

                // If launcher is in closed-loop velocity mode (desiredTPS > small epsilon), use velocity gating
                if (desiredTPS > 1.0) {
                    double err = Math.abs(avgVel - desiredTPS);
                    double frac = (desiredTPS > 0) ? (err / desiredTPS) : 1.0;
                    if (frac <= LAUNCHER_TPS_TOLERANCE_FRAC) return true;
                } else {
                    // Otherwise fall back to comparing commanded power vs actual power
                    double leftP = launcher.getLeftLaunchPower();
                    double rightP = launcher.getRightLaunchPower();
                    double avgP = (Math.abs(leftP) + Math.abs(rightP)) / 2.0;
                    double errP = Math.abs(avgP - desiredPower);
                    if (errP <= LAUNCHER_POWER_TOLERANCE) return true;
                }
            } catch (Exception ignored) {}

            try { Thread.sleep(20); } catch (InterruptedException ignored) {}
        }

        return false;
    }
}
