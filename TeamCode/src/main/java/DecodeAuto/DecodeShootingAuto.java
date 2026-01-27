package DecodeAuto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.RevolverSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * DecodeShootingAuto v4.0 - FULLY DIRECT CONTROL
 * 
 * This version bypasses the RevolverSubsystem state machine entirely
 * and uses direct motor/servo control for reliable autonomous operation.
 * 
 * Key timings:
 * - TICKS_PER_SLOT = 96 (60 degrees)
 * - Shooter spin-up: 1.0 second
 * - Kicker extend: 0.6 second
 * - Kicker retract: 0.4 second
 * - Indexer settle: 0.5 second
 */
@Config
@Autonomous(name = "Decode Shooting Auto v4.0", group = "DecodeAuto")
public class DecodeShootingAuto extends LinearOpMode {

    // === CONFIGURABLE POSITION ===
    public static double START_X = -55;
    public static double START_Y = -55;
    public static double START_HEADING_DEG = 55;

    // === TIMING CONFIGURATION (milliseconds) ===
    public static long SHOOTER_SPINUP_MS = 1000;
    public static long KICKER_EXTEND_MS = 600;
    public static long KICKER_RETRACT_MS = 400;
    public static long INDEXER_SETTLE_MS = 500;
    public static long INTAKE_PULSE_MS = 800; // Time intake runs per ball
    public static long INTAKE_PAUSE_MS = 1200; // INCREASED: Wait for indexer to create empty slot

    // === POWER SETTINGS ===
    public static double SHOOTER_POWER = 0.4; // Using setShooterPowerDirect (raw PWM) to avoid velocity PID max-speed
                                              // issue
    public static double INTAKE_POWER = 1.0;

    // === SUBSYSTEMS ===
    private MecanumDrive drive;
    private RevolverSubsystem revolver;
    private AprilTagNavigator tagNavigator;

    // === STATE ===
    private TagConfiguration.RandomizationPattern detectedPattern = TagConfiguration.RandomizationPattern.UNKNOWN;
    private int collectedBalls = 0; // Track how many balls we actually collected

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize
        Pose2d startPose = new Pose2d(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
        drive = new MecanumDrive(hardwareMap, startPose);
        revolver = new RevolverSubsystem(hardwareMap);
        tagNavigator = new AprilTagNavigator(hardwareMap, "webCam1");

        // Ensure kicker is retracted
        revolver.kickerRetract();

        // Build trajectories
        Action moveToAprilTag = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(-37, -13), Math.toRadians(160))
                .build();

        // Shooting position with BACKUP for better angle
        Action moveToShooting1 = drive.actionBuilder(new Pose2d(-37, -13, Math.toRadians(160)))
                .strafeToLinearHeading(new Vector2d(-35, -31), Math.toRadians(230)) // Back up 5 units
                .build();

        // Move to intake starting position
        Action moveToIntakeStart = drive.actionBuilder(new Pose2d(-35, -31, Math.toRadians(230)))
                .strafeToLinearHeading(new Vector2d(-11, -20), Math.toRadians(278))
                .build();

        Action moveToShooting2 = drive.actionBuilder(new Pose2d(-11, -20, Math.toRadians(278)))
                .strafeToLinearHeading(new Vector2d(-35, -31), Math.toRadians(230)) // Same backup position
                .build();

        Action moveToPark = drive.actionBuilder(new Pose2d(-35, -31, Math.toRadians(230))) // Updated start pos
                .strafeToLinearHeading(new Vector2d(-5, -53), Math.toRadians(90))
                .build();

        // Init telemetry
        telemetry.addData("Mode", "v4.0 DIRECT CONTROL");
        telemetry.addData("Status", "Ready");
        telemetry.addData("Camera", "Waiting for stream...");
        telemetry.update();

        // CRITICAL: Wait for camera to start streaming
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Camera Status", tagNavigator.visionPortal.getCameraState());
            telemetry.addData("Detections", tagNavigator.aprilTag.getDetections().size());
            telemetry.update();
            sleep(20);
        }

        waitForStart();
        if (isStopRequested())
            return;

        // ========================================
        // STEP 1-2: Move & Detect AprilTag
        // ========================================
        showStep("1-2", "AprilTag Detection");
        Actions.runBlocking(new ParallelAction(moveToAprilTag, new DetectPatternAction()));

        // Display result prominently
        telemetry.clearAll();
        telemetry.addLine("=== APRILTAG RESULT ===");
        telemetry.addData("Pattern", detectedPattern.toString());
        telemetry.addData("Tag ID", tagNavigator.getLastDetectedTagId());
        telemetry.addData("Camera State", tagNavigator.visionPortal.getCameraState());
        telemetry.addData("Total Detections", tagNavigator.aprilTag.getDetections().size());

        // Show all detected tags for debugging
        for (int i = 0; i < tagNavigator.aprilTag.getDetections().size(); i++) {
            AprilTagDetection det = tagNavigator.aprilTag.getDetections().get(i);
            telemetry.addData("  Detected Tag " + i, "ID=" + det.id);
        }

        telemetry.update();
        sleep(2000); // Longer pause to review results

        // ========================================
        // STEP 2-3: Move to shooting position
        // ========================================
        showStep("2-3", "Moving to Shoot");
        Actions.runBlocking(moveToShooting1);

        // ========================================
        // STEP 3: Shoot 3 preloaded artifacts
        // ========================================
        showStep("3", "SHOOTING PRELOADED");

        // Start shooter motor
        revolver.setShooterPowerDirect(SHOOTER_POWER);
        sleep(SHOOTER_SPINUP_MS);

        for (int i = 0; i < 3; i++) {
            telemetry.addData("SHOOTING", "Ball %d/3", i + 1);
            telemetry.addData("Indexer Pos", revolver.getIndexerPosition());
            telemetry.update();

            // Kick!
            revolver.kickerEject();
            sleep(KICKER_EXTEND_MS);

            // Retract
            revolver.kickerRetract();
            sleep(KICKER_RETRACT_MS);

            // Index to next slot (except on last shot)
            if (i < 2) {
                revolver.indexerNextSlot();
                sleep(INDEXER_SETTLE_MS);
            }
        }

        // Stop shooter
        revolver.setShooterPowerDirect(0);

        telemetry.addData("STEP 3", "COMPLETE - 3 shots fired");
        telemetry.update();

        // ========================================
        // STEP 4: Move to intake area
        // ========================================
        showStep("4", "Moving to intake area");
        Actions.runBlocking(moveToIntakeStart);

        // ========================================
        // STEP 5: SMART INTAKE (pauses for ball collection)
        // ========================================
        showStep("5", "Smart Intake - collecting balls");
        // SmartIntakeAction handles:
        // - Forward movement while scanning for balls
        // - PAUSING when ball detected to allow indexing
        // - Return to start after collection complete
        Actions.runBlocking(new SmartIntakeAction(3));

        // Note: SmartIntakeAction returns robot to (-11, -30), so we need to
        // move back to shooting position from there
        showStep("6", "Return to Shoot");
        Actions.runBlocking(moveToShooting2);

        // ========================================
        // STEP 6: Shoot ONLY what we actually collected
        // ========================================
        int toShoot = Math.min(collectedBalls, 3); // Cap at 3
        showStep("6", "SHOOTING " + toShoot + " COLLECTED");

        if (toShoot > 0) {
            // Start shooter
            revolver.setShooterPowerDirect(SHOOTER_POWER);
            sleep(SHOOTER_SPINUP_MS);

            for (int i = 0; i < toShoot; i++) {
                telemetry.addData("SHOOTING", "Collected %d/%d", i + 1, toShoot);
                telemetry.update();

                revolver.kickerEject();
                sleep(KICKER_EXTEND_MS);

                revolver.kickerRetract();
                sleep(KICKER_RETRACT_MS);

                if (i < toShoot - 1) {
                    revolver.indexerNextSlot();
                    sleep(INDEXER_SETTLE_MS);
                }
            }

            revolver.setShooterPowerDirect(0);
        } else {
            telemetry.addData("STEP 6", "No balls collected - skipping");
            telemetry.update();
        }

        // ========================================
        // STEP 7: Park
        // ========================================
        showStep("6-7", "Parking");
        Actions.runBlocking(moveToPark);

        // Done
        telemetry.clearAll();
        telemetry.addLine("=== COMPLETE ===");
        telemetry.addData("Pattern", detectedPattern);
        telemetry.update();

        tagNavigator.stop();

        while (opModeIsActive() && !isStopRequested()) {
            sleep(50);
        }
    }

    private void showStep(String step, String desc) {
        telemetry.addData("STEP", step + ": " + desc);
        telemetry.update();
    }

    // =========================================================================
    // ACTIONS
    // =========================================================================

    private class DetectPatternAction implements Action {
        private long startTime = 0;
        private static final long TIMEOUT_MS = 3000;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (startTime == 0)
                startTime = System.currentTimeMillis();

            TagConfiguration.RandomizationPattern pattern = tagNavigator.detectRandomizationPattern();
            int detectionCount = tagNavigator.aprilTag.getDetections().size();

            packet.put("Tag ID", tagNavigator.getLastDetectedTagId());
            packet.put("Detections", detectionCount);

            if (pattern != TagConfiguration.RandomizationPattern.UNKNOWN) {
                detectedPattern = pattern;
                packet.put("Pattern", pattern.toString());
                return false;
            }

            if (System.currentTimeMillis() - startTime > TIMEOUT_MS) {
                packet.put("Pattern", "TIMEOUT - " + detectionCount + " seen");
                return false;
            }

            packet.put("Pattern", "Scanning... (" + detectionCount + " tags)");
            return true;
        }
    }

    /**
     * Intake action that runs intake motor and uses COLOR SENSOR to detect
     * when a ball is actually collected before indexing.
     * 
     * Flow: Run intake -> Wait for color sensor -> Stop intake -> Index -> Repeat
     */
    private class IntakeWithIndexingAction implements Action {
        private final int targetBalls;
        private int ballsCollected = 0;
        private int phase = 0; // 0=intake+detect, 1=index wait
        private long phaseStartTime = 0;
        private long maxIntakeTime = 5000; // Max time to wait for a ball

        public IntakeWithIndexingAction(int targetBalls) {
            this.targetBalls = targetBalls;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (phaseStartTime == 0)
                phaseStartTime = System.currentTimeMillis();

            long elapsed = System.currentTimeMillis() - phaseStartTime;

            // Get LIVE color sensor reading (not cached)
            RevolverSubsystem.SlotColor detected = revolver.readColorNow();
            packet.put("Color", detected.toString());
            packet.put("Distance", String.format("%.1f cm", revolver.getDistance()));

            switch (phase) {
                case 0: // Intake running, waiting for ball detection
                    revolver.setIntakePowerDirect(INTAKE_POWER);
                    packet.put("Intake", "Running - waiting for ball...");

                    // Check if color sensor detects a ball (GREEN or PURPLE)
                    boolean ballDetected = (detected == RevolverSubsystem.SlotColor.GREEN ||
                            detected == RevolverSubsystem.SlotColor.PURPLE);

                    if (ballDetected) {
                        // Ball detected! STOP intake immediately
                        revolver.setIntakePowerDirect(0);
                        packet.put("Intake", "Ball detected! Stopping intake...");
                        phase = 1;
                        phaseStartTime = System.currentTimeMillis();
                    } else if (elapsed >= maxIntakeTime) {
                        // Timeout - move on
                        packet.put("Intake", "Timeout - no ball detected");
                        revolver.setIntakePowerDirect(0);
                        return false;
                    }
                    break;

                case 1: // Short pause to let ball settle before indexing
                    revolver.setIntakePowerDirect(0);
                    packet.put("Intake", "Ball settling...");

                    if (elapsed >= 300) { // 300ms for ball to settle
                        // Now start indexing to create empty slot
                        revolver.indexerNextSlot();
                        ballsCollected++;
                        collectedBalls++;
                        phase = 2;
                        phaseStartTime = System.currentTimeMillis();
                    }
                    break;

                case 2: // Wait for indexer to reach position (creates empty slot)
                    revolver.setIntakePowerDirect(0);
                    packet.put("Intake", "Indexing to empty slot... (" + ballsCollected + "/" + targetBalls + ")");

                    // Wait for indexer to finish moving
                    boolean indexerDone = revolver.isIndexerAtTarget() || elapsed >= INTAKE_PAUSE_MS;

                    if (indexerDone) {
                        if (ballsCollected >= targetBalls) {
                            packet.put("Intake", "Complete: " + ballsCollected + " balls");
                            return false;
                        }
                        // Continue to next ball - now there's an empty slot!
                        phase = 0;
                        phaseStartTime = System.currentTimeMillis();
                    }
                    break;
            }

            packet.put("Balls", ballsCollected + "/" + targetBalls);
            packet.put("Indexer", revolver.getIndexerPosition());
            return true;
        }
    }

    /**
     * Continuous intake action - KEEPS INTAKE RUNNING until timeout.
     * This action NEVER stops the intake early. It runs for the full duration
     * of the parallel trajectory or until MAX_ACTION_TIME_MS timeout.
     * 
     * Ball detection is done for indexing/counting but does NOT stop the intake.
     */
    private class ContinuousIntakeAction implements Action {
        private final int targetBalls;
        private int ballsCollected = 0;
        private boolean indexing = false;
        private long indexStartTime = 0;
        private long actionStartTime = 0;
        private static final long MAX_ACTION_TIME_MS = 15000; // 15 seconds - longer than trajectory
        private RevolverSubsystem.SlotColor lastDetected = RevolverSubsystem.SlotColor.EMPTY;

        public ContinuousIntakeAction(int targetBalls) {
            this.targetBalls = targetBalls;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // *** ALWAYS KEEP INTAKE RUNNING - this is the whole point! ***
            revolver.setIntakePowerDirect(INTAKE_POWER);

            if (actionStartTime == 0) {
                actionStartTime = System.currentTimeMillis();
            }

            long totalElapsed = System.currentTimeMillis() - actionStartTime;

            // TIMEOUT - this is the ONLY way to stop (or when ParallelAction trajectory
            // ends)
            if (totalElapsed >= MAX_ACTION_TIME_MS) {
                revolver.setIntakePowerDirect(0);
                telemetry.addData("INTAKE", "TIMEOUT after %ds, %d balls", totalElapsed / 1000, ballsCollected);
                telemetry.update();
                return false;
            }

            // Read sensor for detection
            RevolverSubsystem.SlotColor detected = revolver.readColorNow();

            // Ball detection and indexing (but DON'T stop intake!)
            if (indexing) {
                long elapsed = System.currentTimeMillis() - indexStartTime;
                if (revolver.isIndexerAtTarget() || elapsed >= INTAKE_PAUSE_MS) {
                    indexing = false;
                    lastDetected = RevolverSubsystem.SlotColor.EMPTY;
                    // Note: We do NOT return false here anymore - keep going!
                }
            } else {
                boolean ballDetected = (detected == RevolverSubsystem.SlotColor.GREEN ||
                        detected == RevolverSubsystem.SlotColor.PURPLE);

                if (ballDetected && lastDetected == RevolverSubsystem.SlotColor.EMPTY) {
                    // Ball detected - index if we haven't hit target yet
                    if (ballsCollected < targetBalls) {
                        revolver.indexerNextSlot();
                        ballsCollected++;
                        collectedBalls++;
                        indexing = true;
                        indexStartTime = System.currentTimeMillis();
                    }
                }
                lastDetected = detected;
            }

            // Telemetry every 500ms
            if (totalElapsed % 500 < 50) {
                telemetry.addData("INTAKE", "RUNNING (%ds)", totalElapsed / 1000);
                telemetry.addData("Balls", "%d/%d (target)", ballsCollected, targetBalls);
                telemetry.addData("Color", detected.toString());
                telemetry.addData("Indexing", indexing ? "YES" : "no");
                telemetry.update();
            }

            // ALWAYS return true - let trajectory completion or timeout end the action
            return true;
        }
    }

    /**
     * Smart Intake Action - Coordinates robot movement with ball collection.
     * 
     * Unlike the previous ParallelAction approach, this action PAUSES movement
     * when a ball is detected, waits for indexing to complete, then resumes.
     * 
     * STATE MACHINE:
     * MOVING_FORWARD → Ball Detected → PAUSED_FOR_INDEX → Index Complete →
     * MOVING_FORWARD
     * MOVING_FORWARD → Target Reached OR Balls Collected → RETURNING
     */
    private class SmartIntakeAction implements Action {
        // State constants (enum not allowed in inner class)
        // Robot starts at (-11, -30) so no need for MOVING_TO_START
        private static final int STATE_MOVING_FORWARD = 0;
        private static final int STATE_PAUSED_FOR_INDEX = 1;
        private static final int STATE_RETURNING = 2;
        private static final int STATE_DONE = 3;

        private final int targetBalls;
        private final double startY = -20; // Updated by user calibration
        private final double endY = -55;
        private final double moveSpeed = 0.1; // Calibrated for reliable movement
        private final double targetHeading = Math.toRadians(278); // Target heading during intake
        private final double headingKp = 2.0; // Proportional gain for heading correction

        private int state = STATE_MOVING_FORWARD; // Start scanning immediately
        private int ballsCollected = 0;
        private long indexStartTime = 0;
        private long actionStartTime = 0;
        private RevolverSubsystem.SlotColor lastDetected = RevolverSubsystem.SlotColor.EMPTY;

        public SmartIntakeAction(int targetBalls) {
            this.targetBalls = targetBalls;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (actionStartTime == 0) {
                actionStartTime = System.currentTimeMillis();
            }

            long elapsed = System.currentTimeMillis() - actionStartTime;

            // Safety timeout: 20 seconds max
            if (elapsed >= 20000) {
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                revolver.setIntakePowerDirect(0);
                telemetry.addData("Smart Intake", "TIMEOUT");
                telemetry.update();
                return false;
            }

            // Get current robot pose
            Pose2d currentPose = drive.localizer.getPose();
            double currentY = currentPose.position.y;

            // Read color sensor
            RevolverSubsystem.SlotColor detected = revolver.readColorNow();
            boolean ballDetected = (detected == RevolverSubsystem.SlotColor.GREEN ||
                    detected == RevolverSubsystem.SlotColor.PURPLE);

            // Telemetry header
            String stateNames[] = { "MOVING_FWD", "PAUSED", "RETURNING", "DONE" };
            telemetry.addData("State", stateNames[state]);
            telemetry.addData("Position", "(%.1f, %.1f)", currentPose.position.x, currentY);
            telemetry.addData("Balls", "%d/%d", ballsCollected, targetBalls);
            telemetry.addData("Color", detected.toString());

            // State machine
            switch (state) {
                case STATE_MOVING_FORWARD:
                    revolver.setIntakePowerDirect(INTAKE_POWER);

                    // Check for ball detection (edge trigger)
                    if (ballDetected && lastDetected == RevolverSubsystem.SlotColor.EMPTY) {
                        // BALL DETECTED! Pause movement
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        revolver.indexerNextSlot();
                        ballsCollected++;
                        collectedBalls++;
                        state = STATE_PAUSED_FOR_INDEX;
                        indexStartTime = System.currentTimeMillis();
                        telemetry.addLine("*** BALL DETECTED - PAUSED ***");
                    }
                    // Check if reached end or collected all balls
                    else if (currentY <= endY + 2) { // Reached end
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        state = STATE_RETURNING;
                        telemetry.addLine("Reached end of path, returning");
                    } else if (ballsCollected >= targetBalls) { // Collected enough
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        state = STATE_RETURNING;
                        telemetry.addLine("Target balls collected, returning");
                    } else {
                        // Continue moving forward with heading correction
                        double headingError = targetHeading - currentPose.heading.toDouble();
                        // Normalize error to [-PI, PI]
                        while (headingError > Math.PI)
                            headingError -= 2 * Math.PI;
                        while (headingError < -Math.PI)
                            headingError += 2 * Math.PI;

                        double turnPower = headingError * headingKp;

                        drive.setDrivePowers(new PoseVelocity2d(
                                new Vector2d(moveSpeed, 0), turnPower));
                        telemetry.addData("[MOVING FWD]", "Y=%.1f Hdg=%.1f°",
                                currentY, Math.toDegrees(currentPose.heading.toDouble()));
                        telemetry.addData("Heading Err", "%.1f° Turn=%.2f",
                                Math.toDegrees(headingError), turnPower);
                    }
                    break;

                case STATE_PAUSED_FOR_INDEX:
                    // Keep intake running even while paused
                    revolver.setIntakePowerDirect(INTAKE_POWER);

                    long indexElapsed = System.currentTimeMillis() - indexStartTime;
                    if (revolver.isIndexerAtTarget() || indexElapsed >= INTAKE_PAUSE_MS) {
                        // Index complete, resume movement or return
                        lastDetected = RevolverSubsystem.SlotColor.EMPTY;

                        if (ballsCollected >= targetBalls || currentY <= endY + 2) {
                            state = STATE_RETURNING;
                            telemetry.addLine("Indexing done, returning");
                        } else {
                            state = STATE_MOVING_FORWARD;
                            telemetry.addLine("Indexing done, resuming");
                        }
                    }
                    break;

                case STATE_RETURNING:
                    revolver.setIntakePowerDirect(0); // Stop intake during return

                    double distToReturn = Math.abs(currentY - startY);
                    telemetry.addData("[RETURNING]", "Dist=%.1f Y=%.1f->%.1f", distToReturn, currentY, startY);

                    if (distToReturn < 3 || currentY >= startY - 1) { // Increased tolerance + upper bound
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        state = STATE_DONE;
                        telemetry.addLine("*** RETURN COMPLETE - STOPPED ***");
                    } else {
                        // Move back (negative X in robot frame = positive Y in field)
                        drive.setDrivePowers(new PoseVelocity2d(
                                new Vector2d(-0.3, 0), 0)); // Faster return, moving backward
                    }
                    break;

                case STATE_DONE:
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                    revolver.setIntakePowerDirect(0);
                    return false; // Action complete
            }

            // Update lastDetected for edge detection
            if (!ballDetected) {
                lastDetected = RevolverSubsystem.SlotColor.EMPTY;
            } else {
                lastDetected = detected;
            }

            // Telemetry already added above, just update
            telemetry.update();

            return true;
        }
    }
}
