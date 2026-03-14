package org.firstinspires.ftc.teamcode.season2025.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.season2025.Components.Turntable;

@Autonomous(name="TurntableMappingTest", group="testing")
public class TurntableMappingTest extends LinearOpMode {

    private Turntable turntable;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Turntable Mapping Test - init");
        telemetry.update();

        turntable = new Turntable(hardwareMap, telemetry);

        telemetry.addLine("Press start to run mapping test");
        telemetry.update();
        waitForStart();

        // Move to logical slots 1..3 and report what the turntable reports after each move.
        for (int logical = 1; logical <= 3 && opModeIsActive(); logical++) {
            telemetry.addData("Test", "Moving to logical slot %d", logical);
            telemetry.update();
            turntable.moveToIndex(logical);
            // wait for physical settle (turntable internally uses a short settle thread)
            sleep(800);
            int reported = turntable.currentSlot();
            double pos = turntable._currentPosition; // note: field is public in Turntable
            telemetry.addData("Result", "logical=%d reported=%d pos=%.3f", logical, reported, pos);
            telemetry.update();
            sleep(800);
        }

        telemetry.addLine("Mapping test complete");
        telemetry.update();
        while (opModeIsActive()) idle();
    }
}
