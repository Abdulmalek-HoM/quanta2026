package DecodeAuto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.RevolverSubsystem;

/**
 * IntakeDebugOpMode - FULLY AUTOMATIC intake testing.
 * 
 * PURPOSE:
 * Test the automatic intake + color sensor + indexer logic in isolation.
 * This uses the EXACT same logic that will be used in autonomous mode.
 * 
 * OPERATION:
 * 1. Press START
 * 2. Intake runs automatically at full power
 * 3. When color sensor detects a ball (GREEN or PURPLE), indexer rotates to
 * next slot
 * 4. Waits for indexer to settle, then resumes detection
 * 5. Stops after collecting 3 balls or timeout (30 seconds)
 * 
 * TELEMETRY shows all sensor data for debugging.
 */
@Config
@Autonomous(name = "Intake Debug (AUTO)", group = "Debug")
public class IntakeDebugOpMode extends LinearOpMode {

    // === CONFIGURABLE VIA FTC DASHBOARD ===
    public static double INTAKE_POWER = 1.0;
    public static int TARGET_BALLS = 3;
    public static long INDEX_PAUSE_MS = 1200; // Wait after ball detected
    public static long MAX_RUNTIME_MS = 300000; // 5 minutes (effectively unlimited for testing)

    private RevolverSubsystem revolver;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        revolver = new RevolverSubsystem(hardwareMap);

        telemetry.addLine("=== INTAKE DEBUG (AUTOMATIC) ===");
        telemetry.addLine("");
        telemetry.addLine("This mode will:");
        telemetry.addLine("1. Run intake automatically");
        telemetry.addLine("2. Detect balls via color sensor");
        telemetry.addLine("3. Index to next slot on detection");
        telemetry.addLine("4. Stop after 3 balls (no timeout)");
        telemetry.addLine("");
        telemetry.addLine("Press START to begin...");
        telemetry.update();

        waitForStart();

        // === STATE MACHINE VARIABLES ===
        int ballsCollected = 0;
        boolean indexing = false;
        long indexStartTime = 0;
        long startTime = System.currentTimeMillis();
        RevolverSubsystem.SlotColor lastDetected = RevolverSubsystem.SlotColor.EMPTY;

        // === MAIN LOOP ===
        while (opModeIsActive()) {
            long elapsed = System.currentTimeMillis() - startTime;

            // === TIMEOUT CHECK ===
            if (elapsed >= MAX_RUNTIME_MS) {
                revolver.setIntakePowerDirect(0);
                telemetry.addLine("*** TIMEOUT ***");
                telemetry.addData("Balls Collected", ballsCollected);
                telemetry.update();
                sleep(3000);
                break;
            }

            // === COMPLETION CHECK ===
            if (ballsCollected >= TARGET_BALLS && !indexing) {
                revolver.setIntakePowerDirect(0);
                telemetry.addLine("*** COMPLETE! ***");
                telemetry.addData("Balls Collected", ballsCollected);
                telemetry.update();
                sleep(3000);
                break;
            }

            // === ALWAYS RUN INTAKE ===
            revolver.setIntakePowerDirect(INTAKE_POWER);

            // === READ COLOR SENSOR (LIVE) ===
            RevolverSubsystem.SlotColor detected = revolver.readColorNow();
            double distance = revolver.getDistance();
            int r = revolver.getRed();
            int g = revolver.getGreen();
            int b = revolver.getBlue();

            // === STATE MACHINE ===
            String status;

            if (indexing) {
                // Waiting for indexer to complete
                long indexElapsed = System.currentTimeMillis() - indexStartTime;
                status = String.format("INDEXING... (%d ms)", indexElapsed);

                if (revolver.isIndexerAtTarget() || indexElapsed >= INDEX_PAUSE_MS) {
                    indexing = false;
                    lastDetected = RevolverSubsystem.SlotColor.EMPTY; // Reset for next detection
                    status = "INDEX COMPLETE - resuming detection";
                }
            } else {
                // Looking for balls
                boolean ballDetected = (detected == RevolverSubsystem.SlotColor.GREEN ||
                        detected == RevolverSubsystem.SlotColor.PURPLE);

                // Edge detection: only trigger on EMPTY -> DETECTED transition
                if (ballDetected && lastDetected == RevolverSubsystem.SlotColor.EMPTY) {
                    // BALL DETECTED!
                    status = "*** BALL DETECTED! Indexing... ***";
                    revolver.indexerNextSlot();
                    ballsCollected++;
                    indexing = true;
                    indexStartTime = System.currentTimeMillis();
                } else if (ballDetected) {
                    status = "Ball still in sensor (waiting for clear)";
                } else {
                    status = "Scanning for balls...";
                }

                lastDetected = detected;
            }

            // === TELEMETRY ===
            telemetry.addLine("========== INTAKE DEBUG (AUTO) ==========");
            telemetry.addData("Runtime", "%.1f / %.1f sec", elapsed / 1000.0, MAX_RUNTIME_MS / 1000.0);
            telemetry.addData("Status", status);
            telemetry.addLine("");

            telemetry.addLine("--- COLOR SENSOR ---");
            telemetry.addData("Detected", detected.toString());
            telemetry.addData("Distance", "%.2f cm", distance);
            telemetry.addData("RGB", "R:%d  G:%d  B:%d", r, g, b);
            telemetry.addLine("");

            telemetry.addLine("--- INDEXER ---");
            telemetry.addData("Position", revolver.getIndexerPosition());
            telemetry.addData("At Target", revolver.isIndexerAtTarget());
            telemetry.addData("Indexing", indexing);
            telemetry.addLine("");

            telemetry.addLine("--- INVENTORY ---");
            telemetry.addData("Balls Collected", "%d / %d", ballsCollected, TARGET_BALLS);
            telemetry.addData("Last Detected", lastDetected.toString());
            telemetry.addLine("");

            telemetry.addData("Intake Power", "%.0f%%", INTAKE_POWER * 100);
            telemetry.update();
        }

        // Cleanup
        revolver.setIntakePowerDirect(0);
    }
}
