package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//@TeleOp(name="Dropdown Menu with Highlight")
public class DropdownMenuHighlight extends LinearOpMode {

    private final String[] motifs = {"PPG", "PGP", "GPP"};
    private int index = 0;
    private boolean isSelected = false;

    // Edge detection
    private boolean prevUp = false;
    private boolean prevDown = false;
    private boolean prevRight = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // INIT LOOP
        while (opModeInInit()) {

            boolean up    = gamepad1.dpad_up;
            boolean down  = gamepad1.dpad_down;
            boolean right = gamepad1.dpad_right;

            // Scroll up
            if (up && !prevUp) {
                index--;
                if (index < 0) index = motifs.length - 1;
            }

            // Scroll down
            if (down && !prevDown) {
                index++;
                if (index >= motifs.length) index = 0;
            }

            // Select
            if (right && !prevRight) {
                isSelected = true;
            }

            // Update edge states
            prevUp = up;
            prevDown = down;
            prevRight = right;

            // TELEMETRY UI
            telemetry.addLine("=== Motif Selection Menu ===");

            if (!isSelected) {
                // Show all options with highlight arrow
                for (int i = 0; i < motifs.length; i++) {
                    if (i == index) {
                        telemetry.addLine("> " + motifs[i]);
                    } else {
                        telemetry.addLine("  " + motifs[i]);
                    }
                }
                telemetry.addLine("Press RIGHT to select");
            } else {
                telemetry.addLine("---- FINAL SELECTION ----");
                telemetry.addData("Chosen Motif", motifs[index]);
            }

            telemetry.update();
        }

        // Save final selection
        String finalMotif = motifs[index];

        waitForStart();

        // After start
        telemetry.addData("Selected Option", finalMotif);
        telemetry.update();

        while (opModeIsActive()) {
            // Your teleop logic here
        }
    }
}
