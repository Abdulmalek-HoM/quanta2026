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
import com.qualcomm.robotcore.hardware.DcMotor;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.RevolverSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * SmartIndexingDecodeShootingAutoRed v1.0 - SMART COLOR-BASED INDEXING (RED SIDE)
 * 
 * Based on DecodeShootingAutoRed v4.0, this version adds smart indexing
 * to reorder balls during travel based on the detected AprilTag pattern:
 * 
 * Tag 21 (GPP): Green first  → Spin 2 slots when green detected
 * Tag 22 (PGP): Green second → Spin 1 slot when green detected
 * Tag 23 (PPG): Green last   → Do nothing (green stays last)
 * 
 * The SmartIndexAction runs as a parallel action while moving to 
 * the shooting position, ensuring no time is lost.
 * 
 * RED SIDE TRANSFORMATIONS:
 * - Y-axis: INVERTED (Y → -Y)
 * - Headings: adjusted for red side orientation
 * - SmartIntakeAction: Movement direction inverted for correct intake direction
 */
@Config
@Autonomous(name = "Smart Indexing Decode Auto RED v1.0", group = "DecodeAuto")
public class SmartIndexingDecodeShootingAutoRed extends LinearOpMode {

    // === CONFIGURABLE POSITION (MIRRORED: Y inverted) ===
    public static double START_X = -56;
    public static double START_Y = 47; // INVERTED from -47
    public static double START_HEADING_DEG = -55;

    // === TIMING CONFIGURATION (milliseconds) ===
    public static long SHOOTER_SPINUP_MS = 1200;
    public static long KICKER_EXTEND_MS = 600;
    public static long KICKER_RETRACT_MS = 400;
    public static long INDEXER_SETTLE_MS = 500;
    public static long INTAKE_PULSE_MS = 800;
    public static long INTAKE_PAUSE_MS = 1200;

    // === POWER SETTINGS ===
    public static double SHOOTER_POWER = 0.7;
    public static double INTAKE_POWER = 1.0;

    // === SUBSYSTEMS ===
    private MecanumDrive drive;
    private RevolverSubsystem revolver;
    private AprilTagNavigator tagNavigator;

    // === STATE ===
    private TagConfiguration.RandomizationPattern detectedPattern = TagConfiguration.RandomizationPattern.UNKNOWN;
    private int collectedBalls = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize
        Pose2d startPose = new Pose2d(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
        drive = new MecanumDrive(hardwareMap, startPose);
        revolver = new RevolverSubsystem(hardwareMap);
        tagNavigator = new AprilTagNavigator(hardwareMap, "webCam1");

        // Ensure kicker is retracted
        revolver.kickerRetract();

        // SYNC SUBSYSTEM CONFIG WITH AUTO CONSTANTS
        revolver.fsmShooterPower = SHOOTER_POWER;
        revolver.fsmIntakePower = INTAKE_POWER;
        revolver.disableFSMKickerControl = false;

        // Build trajectories (MIRRORED for RED side)
        Action moveToAprilTag = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(-43, 31), Math.toRadians(215))
                .build();

        Action moveToShooting1 = drive.actionBuilder(new Pose2d(-43, 31, Math.toRadians(215)))
                .strafeToLinearHeading(new Vector2d(-41, 33), Math.toRadians(115))
                .build();

        Action moveToIntakeStart = drive.actionBuilder(new Pose2d(  -41, 33, Math.toRadians(115)))
                .strafeTo(new Vector2d(-10, 7))
                .turnTo(Math.toRadians(90))
                .build();

        Action moveToShooting2 = drive.actionBuilder(new Pose2d(-10, 7, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-41, 33), Math.toRadians(115))
                .build();

        Action moveToPark = drive.actionBuilder(new Pose2d(-41, 33, Math.toRadians(115)))
                .strafeToLinearHeading(new Vector2d(-5, 30), Math.toRadians(270))
                .build();

        // Init telemetry
        telemetry.addData("Mode", "v1.0 RED SMART INDEXING");
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
        sleep(500);

        // ========================================
        // STEP 2-3: Move to shooting position
        // ========================================
        showStep("2-3", "Moving to Shoot");
        Actions.runBlocking(moveToShooting1);

        // ========================================
        // STEP 3: Shoot 3 preloaded artifacts
        // ========================================
        showStep("3", "SHOOTING PRELOADED");

        revolver.setShooterPowerDirect(SHOOTER_POWER);
        sleep(SHOOTER_SPINUP_MS);

        for (int i = 0; i < 3; i++) {
            telemetry.addData("SHOOTING", "Ball %d/3", i + 1);
            telemetry.addData("Indexer Pos", revolver.getIndexerPosition());
            telemetry.update();

            revolver.kickerEject();
            sleep(KICKER_EXTEND_MS);

            revolver.kickerRetract();
            sleep(KICKER_RETRACT_MS);

            if (i < 2) {
                revolver.indexerNextSlot();
                sleep(INDEXER_SETTLE_MS);
            }
        }

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
        Actions.runBlocking(new SmartIntakeActionRed(3));

        // ========================================
        // STEP 5.5: SMART INDEXING (parallel with movement)
        // ========================================
        showStep("5.5-6", "Smart Index + Return to Shoot");
        Actions.runBlocking(new ParallelAction(moveToShooting2, new SmartIndexAction()));

        // ========================================
        // STEP 6: Shoot ONLY what we actually collected
        // ========================================
        int toShoot = Math.min(collectedBalls, 3);
        showStep("6", "SHOOTING " + toShoot + " COLLECTED");

        if (toShoot > 0) {
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
        telemetry.addData("Shooter Power", SHOOTER_POWER);
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
     * SMART INDEXING ACTION v2.0 - Reorders balls based on detected AprilTag pattern.
     * 
     * FIXES in v2.0:
     * - Added STARTUP_DELAY_MS: Wait for balls to settle before scanning
     * - Added DEBOUNCE: Require 3 consecutive green readings to confirm
     * - Added better telemetry for debugging
     * 
     * Tag 21 (GPP): Green first  → Spin 2 slots
     * Tag 22 (PGP): Green second → Spin 1 slot
     * Tag 23 (PPG): Green last   → Do nothing
     */
    private class SmartIndexAction implements Action {
        // State tracking
        private boolean greenConfirmed = false;
        private boolean indexingStarted = false;
        private boolean indexingComplete = false;
        private long indexStartTime = 0;
        private long actionStartTime = 0;
        
        // DEBOUNCE: Count consecutive green readings
        private int greenConsecutiveCount = 0;
        private static final int GREEN_CONFIRM_THRESHOLD = 3; // Need 3 consecutive greens
        
        // Timing constants
        private static final long STARTUP_DELAY_MS = 500;   // Wait for balls to settle
        private static final long MAX_SCAN_TIME_MS = 5000;  // Max time to look for green
        private static final long INDEX_TIMEOUT_MS = 2000;  // Max time to wait for indexer
        
        // Debug tracking
        private int totalReadings = 0;
        private int greenReadings = 0;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (actionStartTime == 0) {
                actionStartTime = System.currentTimeMillis();
                telemetry.addData("SMART INDEX", "Starting - Pattern: %s", detectedPattern.toString());
                telemetry.update();
            }
            
            long elapsed = System.currentTimeMillis() - actionStartTime;
            
            packet.put("SmartIndex State", getStateString());
            packet.put("Pattern", detectedPattern.toString());
            packet.put("Elapsed", elapsed + "ms");
            
            // If pattern is PPG (23), green should be last - do nothing
            if (detectedPattern == TagConfiguration.RandomizationPattern.PURPLE_PURPLE_GREEN) {
                packet.put("SmartIndex", "PPG - No reorder needed");
                telemetry.addData("SMART INDEX", "PPG pattern - skipping (green already last)");
                telemetry.update();
                indexingComplete = true;
                return false;
            }
            
            // If pattern is unknown, skip smart indexing
            if (detectedPattern == TagConfiguration.RandomizationPattern.UNKNOWN) {
                packet.put("SmartIndex", "Unknown pattern - skipping");
                telemetry.addData("SMART INDEX", "UNKNOWN pattern - skipping");
                telemetry.update();
                indexingComplete = true;
                return false;
            }
            
            // Already done
            if (indexingComplete) {
                return false;
            }
            
            // === STARTUP DELAY: Wait for balls to settle ===
            if (elapsed < STARTUP_DELAY_MS) {
                packet.put("SmartIndex", "Waiting for settle... " + elapsed + "/" + STARTUP_DELAY_MS + "ms");
                telemetry.addData("SMART INDEX", "Settling... %d/%dms", elapsed, STARTUP_DELAY_MS);
                telemetry.update();
                return true; // Keep waiting
            }
            
            // === PHASE 1: Scan for green ball with DEBOUNCE ===
            if (!greenConfirmed) {
                // Timeout check (excluding startup delay)
                long scanTime = elapsed - STARTUP_DELAY_MS;
                if (scanTime > MAX_SCAN_TIME_MS) {
                    packet.put("SmartIndex", "TIMEOUT - Green not confirmed");
                    telemetry.addData("SMART INDEX", "TIMEOUT after %d readings (%d green)", totalReadings, greenReadings);
                    telemetry.update();
                    indexingComplete = true;
                    return false;
                }
                
                // Read color sensor
                RevolverSubsystem.SlotColor color = revolver.readColorNow();
                totalReadings++;
                
                packet.put("Detected Color", color.toString());
                packet.put("Green Count", greenConsecutiveCount + "/" + GREEN_CONFIRM_THRESHOLD);
                
                // DEBOUNCE: Count consecutive green readings
                if (color == RevolverSubsystem.SlotColor.GREEN) {
                    greenConsecutiveCount++;
                    greenReadings++;
                    
                    telemetry.addData("SMART INDEX", "Green detected! Count: %d/%d", 
                            greenConsecutiveCount, GREEN_CONFIRM_THRESHOLD);
                    telemetry.update();
                    
                    // CONFIRMED: Got enough consecutive greens
                    if (greenConsecutiveCount >= GREEN_CONFIRM_THRESHOLD) {
                        greenConfirmed = true;
                        int slotsToSpin = getSlotsForPattern();
                        
                        packet.put("SmartIndex", "GREEN CONFIRMED! Spinning " + slotsToSpin + " slots");
                        telemetry.addData("SMART INDEX", "GREEN CONFIRMED! Spinning %d slots for %s", 
                                slotsToSpin, detectedPattern.toString());
                        telemetry.update();
                        
                        if (slotsToSpin > 0) {
                            revolver.moveIndexerSlots(slotsToSpin);
                            indexingStarted = true;
                            indexStartTime = System.currentTimeMillis();
                        } else {
                            // No spinning needed (shouldn't happen for GPP/PGP)
                            indexingComplete = true;
                            return false;
                        }
                    }
                } else {
                    // Not green - reset consecutive counter
                    if (greenConsecutiveCount > 0) {
                        telemetry.addData("SMART INDEX", "Green lost, resetting count (was %d)", greenConsecutiveCount);
                        telemetry.update();
                    }
                    greenConsecutiveCount = 0;
                }
                
                return true; // Keep scanning
            }
            
            // === PHASE 2: Wait for indexer to complete ===
            if (indexingStarted && !indexingComplete) {
                long indexElapsed = System.currentTimeMillis() - indexStartTime;
                
                boolean atTarget = revolver.isIndexerAtTarget();
                packet.put("Indexer At Target", atTarget);
                packet.put("Index Time", indexElapsed + "ms");
                
                if (atTarget || indexElapsed > INDEX_TIMEOUT_MS) {
                    indexingComplete = true;
                    packet.put("SmartIndex", "COMPLETE - Balls reordered");
                    telemetry.addData("SMART INDEX", "COMPLETE! Indexer settled after %dms", indexElapsed);
                    telemetry.update();
                    return false;
                }
                
                packet.put("SmartIndex", "Indexing... " + indexElapsed + "ms");
                telemetry.addData("SMART INDEX", "Indexing... %dms", indexElapsed);
                telemetry.update();
                return true; // Keep waiting
            }
            
            return !indexingComplete;
        }
        
        /**
         * Get the number of slots to spin based on detected pattern.
         * 
         * GPP (21): Green first  → Spin 2 slots to move green to shooting position
         * PGP (22): Green second → Spin 1 slot to move green to second position
         * PPG (23): Green last   → No spin needed (handled above)
         */
        private int getSlotsForPattern() {
            switch (detectedPattern) {
                case GREEN_PURPLE_PURPLE:
                    return 2; // Green needs to be first
                case PURPLE_GREEN_PURPLE:
                    return 1; // Green needs to be second
                case PURPLE_PURPLE_GREEN:
                    return 0; // Green needs to be last (no action)
                default:
                    return 0;
            }
        }
        
        private String getStateString() {
            if (indexingComplete) return "COMPLETE";
            if (indexingStarted) return "INDEXING";
            if (greenConfirmed) return "GREEN_CONFIRMED";
            if (greenConsecutiveCount > 0) return "DETECTING(" + greenConsecutiveCount + ")";
            return "SCANNING";
        }
    }

    /**
     * Smart Intake Action for RED SIDE - Movement direction INVERTED
     * 
     * Key changes from Blue side:
     * - startY: +7 (was -5)
     * - endY: +40 (was -40)
     * - targetHeading: 90° (was 280°)
     * - moveSpeed: POSITIVE to move forward in +Y direction
     * - Return direction: NEGATIVE speed
     * - End condition: currentY >= endY
     * - Return condition: currentY <= startY
     */
    private class SmartIntakeActionRed implements Action {
        private static final int STATE_MOVING_FORWARD = 0;
        private static final int STATE_PAUSED_FOR_INDEX = 1;
        private static final int STATE_RETURNING = 2;
        private static final int STATE_DONE = 3;

        private final int targetBalls;
        private final double startY = 7;
        private final double endY = 40;
        private final double moveSpeed = 0.15;
        private final double targetHeading = Math.toRadians(90);
        private final double headingKp = 2.0;
        private final double strafeKp = 0.15;

        private int state = STATE_MOVING_FORWARD;
        private int ballsCollected = 0;
        private long indexStartTime = 0;
        private long actionStartTime = 0;
        private double targetX = 0;
        private boolean initialized = false;
        private RevolverSubsystem.SlotColor lastDetected = RevolverSubsystem.SlotColor.EMPTY;

        public SmartIntakeActionRed(int targetBalls) {
            this.targetBalls = targetBalls;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                targetX = drive.localizer.getPose().position.x;
                actionStartTime = System.currentTimeMillis();
                initialized = true;
            }

            drive.updatePoseEstimate();

            long elapsed = System.currentTimeMillis() - actionStartTime;

            if (elapsed >= 10000) {
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                revolver.setIntakePowerDirect(0);
                telemetry.addData("Smart Intake", "TIMEOUT");
                telemetry.update();
                return false;
            }

            Pose2d currentPose = drive.localizer.getPose();
            double currentY = currentPose.position.y;
            double currentX = currentPose.position.x;

            // Heading error
            double headingError = targetHeading - currentPose.heading.toDouble();
            while (headingError > Math.PI)
                headingError -= 2 * Math.PI;
            while (headingError < -Math.PI)
                headingError += 2 * Math.PI;

            // Strafe error
            double xError = targetX - currentX;

            // Corrections
            double turnPower = headingError * headingKp;
            // NEGATED for Red side orientation
            double strafeCorrection = -xError * strafeKp;

            // Color sensor
            RevolverSubsystem.SlotColor detected = revolver.readColorNow();
            boolean ballDetected = (detected == RevolverSubsystem.SlotColor.GREEN ||
                    detected == RevolverSubsystem.SlotColor.PURPLE);

            // Telemetry
            String stateNames[] = { "MOVING_FWD", "PAUSED", "RETURNING", "DONE" };
            telemetry.addData("State", stateNames[state]);
            telemetry.addData("Position", "(%.1f, %.1f)", currentX, currentY);
            telemetry.addData("Target X", "%.1f", targetX);
            telemetry.addData("Drift Err", "%.2f (Corr: %.2f)", xError, strafeCorrection);
            telemetry.addData("Color", detected.toString());

            switch (state) {
                case STATE_MOVING_FORWARD:
                    revolver.setIntakePowerDirect(INTAKE_POWER);

                    if (ballDetected && lastDetected == RevolverSubsystem.SlotColor.EMPTY) {
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        revolver.indexerNextSlot();
                        ballsCollected++;
                        collectedBalls++;
                        state = STATE_PAUSED_FOR_INDEX;
                        indexStartTime = System.currentTimeMillis();
                        telemetry.addLine("*** BALL DETECTED - PAUSED ***");
                    }
                    else if (currentY >= endY - 2) {
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        state = STATE_RETURNING;
                        telemetry.addLine("Reached end of path, returning");
                    } else if (ballsCollected >= targetBalls) {
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        state = STATE_RETURNING;
                        telemetry.addLine("Target balls collected, returning");
                    } else {
                        drive.setDrivePowers(new PoseVelocity2d(
                                new Vector2d(moveSpeed, strafeCorrection), turnPower));
                        telemetry.addData("[MOVING FWD]", "Vel=%.2f, Strafe=%.2f", moveSpeed, strafeCorrection);
                    }
                    break;

                case STATE_PAUSED_FOR_INDEX:
                    revolver.setIntakePowerDirect(INTAKE_POWER);

                    long indexElapsed = System.currentTimeMillis() - indexStartTime;
                    if (revolver.isIndexerAtTarget() || indexElapsed >= INTAKE_PAUSE_MS) {
                        lastDetected = RevolverSubsystem.SlotColor.EMPTY;

                        if (ballsCollected >= targetBalls || currentY >= endY - 2) {
                            state = STATE_RETURNING;
                            telemetry.addLine("Indexing done, returning");
                        } else {
                            state = STATE_MOVING_FORWARD;
                            telemetry.addLine("Indexing done, resuming");
                        }
                    }
                    break;

                case STATE_RETURNING:
                    revolver.setIntakePowerDirect(0);

                    double distToReturn = Math.abs(currentY - startY);
                    telemetry.addData("[RETURNING]", "Dist=%.1f", distToReturn);

                    if (distToReturn < 3 || currentY <= startY + 1) {
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        state = STATE_DONE;
                        telemetry.addLine("*** RETURN COMPLETE - STOPPED ***");
                    } else {
                        drive.setDrivePowers(new PoseVelocity2d(
                                new Vector2d(-0.3, strafeCorrection), turnPower));
                    }
                    break;

                case STATE_DONE:
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                    revolver.setIntakePowerDirect(0);
                    return false;
            }

            if (!ballDetected) {
                lastDetected = RevolverSubsystem.SlotColor.EMPTY;
            } else {
                lastDetected = detected;
            }

            telemetry.update();
            return true;
        }
    }
}
