package DecodeAuto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.RevolverSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * DecodeShootingAutoRed v4.0 - OPPOSITE SIDE (RED ALLIANCE)
 * 
 * This is the mirrored version of DecodeShootingAuto for the opposite side of
 * the field.
 * Transformation applied:
 * - Y-axis: INVERTED (Y → -Y)
 * - Headings: +100 degrees to each angle
 * - SmartIntakeAction: Movement direction inverted for correct intake direction
 * 
 * Key timings (same as Blue):
 * - TICKS_PER_SLOT = 96 (60 degrees)
 * - Shooter spin-up: 1.0 second
 * - Kicker extend: 0.6 second
 * - Kicker retract: 0.4 second
 * - Indexer settle: 0.5 second
 */
@Config
@Disabled
@Autonomous(name = "Decode Shooting Auto RED v4.0", group = "DecodeAuto")
public class DecodeShootingAutoRed extends LinearOpMode {

    // === CONFIGURABLE POSITION (MIRRORED: Y inverted, heading +100°) ===
    // Original Blue: (-56, -47, 55°) → Red: (-56, +47, 155°)
    public static double START_X = -56;
    public static double START_Y = 47; // INVERTED from -47
    public static double START_HEADING_DEG = -55; // +100° from 55°

    // === TIMING CONFIGURATION (milliseconds) - same as Blue ===
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

        // Build trajectories (MIRRORED: Y inverted, heading +100°)
        // Original Blue: (-43, -31, 160°) → Red: (-43, +31, 260°)
        Action moveToAprilTag = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(-43, 31), Math.toRadians(215))
                .build();

        // Original Blue: (-41, -33, 240°) → Red: (-41, +33, 340°)
        Action moveToShooting1 = drive.actionBuilder(new Pose2d(-43, 31, Math.toRadians(215)))
                .strafeToLinearHeading(new Vector2d(-41, 33), Math.toRadians(125))
                .build();

        // Original Blue: (-13, -5, 280°) → Red: (-13, +5, 380° = 20°)
        Action moveToIntakeStart = drive.actionBuilder(new Pose2d(-41, 33, Math.toRadians(125)))
                .strafeTo(new Vector2d(-10, 7))
                .turnTo(Math.toRadians(90)) // 280° + 100° = 380° → 20°
                .build();

        // Original Blue: (-41, -33, 240°) → Red: (-41, +33, 340°)
        Action moveToShooting2 = drive.actionBuilder(new Pose2d(-10, 7, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-41, 33), Math.toRadians(125))
                .build();

        // Original Blue: (-5, -30, 90°) → Red: (-5, +30, 190°)
        Action moveToPark = drive.actionBuilder(new Pose2d(-41, 33, Math.toRadians(125)))
                .strafeToLinearHeading(new Vector2d(-5, 30), Math.toRadians(270))
                .build();

        // Init telemetry
        telemetry.addData("Mode", "v4.0 RED ALLIANCE");
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

        showStep("6", "Return to Shoot");
        Actions.runBlocking(moveToShooting2);

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
     * Smart Intake Action for RED SIDE - Movement direction INVERTED
     * 
     * Key changes from Blue side:
     * - startY: +5 (was -5)
     * - endY: +40 (was -40)
     * - targetHeading: 20° (was 280°, +100°)
     * - moveSpeed: POSITIVE to move forward in +Y direction (was negative for -Y)
     * - Return direction: NEGATIVE speed (was positive)
     * - End condition: currentY >= endY (was <=)
     * - Return condition: currentY <= startY (was >=)
     */
    private class SmartIntakeActionRed implements Action {
        private static final int STATE_MOVING_FORWARD = 0;
        private static final int STATE_PAUSED_FOR_INDEX = 1;
        private static final int STATE_RETURNING = 2;
        private static final int STATE_DONE = 3;

        private final int targetBalls;
        // INVERTED from Blue: startY=+5 (was -5), endY=+40 (was -40)
        private final double startY = 7;
        private final double endY = 40;
        private final double moveSpeed = 0.15; // Positive = forward in +Y direction
        // Heading +100°: 280° + 100° = 380° → 20°
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
            // INVERTED for 90° heading: At 90°, +Y strafe moves robot toward +X global
            // which is OPPOSITE to the 280° Blue side where +Y strafe moves toward -X
            // Therefore we NEGATE the strafe correction to correct drift properly
            double strafeCorrection = -xError * strafeKp; // NEGATED for Red side

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
                    // INVERTED: Check currentY >= endY (was <=)
                    else if (currentY >= endY - 2) {
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        state = STATE_RETURNING;
                        telemetry.addLine("Reached end of path, returning");
                    } else if (ballsCollected >= targetBalls) {
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        state = STATE_RETURNING;
                        telemetry.addLine("Target balls collected, returning");
                    } else {
                        // Move forward in +Y direction (positive speed)
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

                        // INVERTED: Check currentY >= endY (was <=)
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

                    // INVERTED: Check currentY <= startY (was >=)
                    if (distToReturn < 3 || currentY <= startY + 1) {
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        state = STATE_DONE;
                        telemetry.addLine("*** RETURN COMPLETE - STOPPED ***");
                    } else {
                        // Move back in -Y direction (negative speed)
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
