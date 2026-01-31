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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Red 2 Smart Auto - Far Side Red Alliance
 * 
 * Coordinates:
 * 1. Start: (60, 24)
 * 2. Shoot: (-57, 18) @ 115 deg
 * 3. Intake: (6, 20) @ 90 deg -> Scan to Y=55
 * 4. Return: (-57, 18) @ 115 deg
 */
@Config
@Autonomous(name = "Red 2 Smart Auto (Far Side)", group = "DecodeAuto")
public class Red2SmartAuto extends LinearOpMode {

    // === CONFIGURABLE POSITION ===
    public static double START_X = 60;
    public static double START_Y = 24;
    public static double START_HEADING_DEG = 180;

    // === TIMING CONFIGURATION (milliseconds) ===
    public static long SHOOTER_SPINUP_MS = 1000;
    public static long KICKER_EXTEND_MS = 500;
    public static long KICKER_RETRACT_MS = 400;
    public static long INDEXER_SETTLE_MS = 700;
    public static long INTAKE_PAUSE_MS = 100;

    // === POWER SETTINGS ===
    public static double SHOOTER_POWER = 0.75;
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

        revolver.kickerRetract();
        revolver.fsmShooterPower = SHOOTER_POWER;
        revolver.fsmIntakePower = INTAKE_POWER;
        revolver.disableFSMKickerControl = false;

        // --- TRAJECTORIES ---

        // 1. Move to Shooting Position (Cross Field) - Split into segments for
        // stability
        // From (60, 24) to (-57, 18)
        Action moveToShooting1 = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(12, 12), Math.toRadians(180)) // Waypoint 1: Clear Truss/Center
                .strafeToLinearHeading(new Vector2d(-57, 20), Math.toRadians(105)) // Target
                .build();

        // 2. Move to Intake Start
        // From (-57, 18) to (6, 20)
        Action moveToIntakeStart = drive.actionBuilder(new Pose2d(-57, 20, Math.toRadians(105)))
                .splineTo(new Vector2d(5, 15), Math.toRadians(90))
                .build();

        // 3. Return to Shooting Position
        // From (6, 20 - after return) to (-57, 18)
        // Note: SmartIntake returns to StartY (20)
        Action moveToShooting2 = drive.actionBuilder(new Pose2d(5, 15, Math.toRadians(90)))
                .splineTo(new Vector2d(-57, 20), Math.toRadians(115))
                .build();

        // Init Telemetry
        telemetry.addData("Mode", "Red 2 FAR SIDE");
        telemetry.addData("Status", "Waiting for Start");
        telemetry.update();

        // Detect Pattern during Init
        while (!isStarted() && !isStopRequested()) {
            TagConfiguration.RandomizationPattern pattern = tagNavigator.detectRandomizationPattern();
            if (pattern != TagConfiguration.RandomizationPattern.UNKNOWN) {
                detectedPattern = pattern;
            }
            telemetry.addData("Pattern", detectedPattern);
            telemetry.addData("Camera", tagNavigator.visionPortal.getCameraState());
            telemetry.update();
            sleep(20);
        }

        waitForStart();
        if (isStopRequested())
            return;

        // ========================================
        // STEP 1: Move to Shooting (+ Re-detect if needed)
        // ========================================
        telemetry.addLine("Moving to Shooting...");
        telemetry.update();

        if (detectedPattern == TagConfiguration.RandomizationPattern.UNKNOWN) {
            Actions.runBlocking(new ParallelAction(moveToShooting1, new DetectPatternAction()));
        } else {
            Actions.runBlocking(moveToShooting1);
        }

        // ========================================
        // STEP 2: Shoot Preloaded
        // ========================================
        shoot(3);

        // ========================================
        // STEP 3: Move to Intake
        // ========================================
        Actions.runBlocking(moveToIntakeStart);

        // ========================================
        // STEP 4: Smart Intake (Y=20 to Y=40)
        // ========================================
        Actions.runBlocking(new SmartIntakeAction(3));

        // ========================================
        // STEP 5: Return to Shoot + Smart Index
        // ========================================
        Actions.runBlocking(new ParallelAction(moveToShooting2, new SmartIndexAction()));

        // ========================================
        // STEP 6: Shoot Collected
        // ========================================
        int toShoot = Math.min(collectedBalls, 3);
        if (toShoot > 0) {
            shoot(toShoot);
        }

        tagNavigator.stop();
    }

    // Helper for shooting
    private void shoot(int count) {
        revolver.setShooterPowerDirect(SHOOTER_POWER);
        sleep(SHOOTER_SPINUP_MS);

        for (int i = 0; i < count; i++) {
            revolver.kickerEject();
            sleep(KICKER_EXTEND_MS);
            revolver.kickerRetract();
            sleep(KICKER_RETRACT_MS);

            if (i < count - 1) {
                revolver.indexerNextSlot();
                sleep(INDEXER_SETTLE_MS);
            }
        }
        revolver.setShooterPowerDirect(0);
    }

    // =========================================================================
    // ACTIONS
    // =========================================================================

    private class DetectPatternAction implements Action {
        private long startTime = 0;
        private static final long TIMEOUT_MS = 2000;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (startTime == 0)
                startTime = System.currentTimeMillis();
            if (detectedPattern != TagConfiguration.RandomizationPattern.UNKNOWN)
                return false;

            TagConfiguration.RandomizationPattern pattern = tagNavigator.detectRandomizationPattern();
            if (pattern != TagConfiguration.RandomizationPattern.UNKNOWN) {
                detectedPattern = pattern;
                return false;
            }

            if (System.currentTimeMillis() - startTime > TIMEOUT_MS)
                return false;
            packet.put("Pattern", "Scanning...");
            return true;
        }
    }

    private class SmartIndexAction implements Action {
        private boolean greenConfirmed = false;
        private boolean indexingStarted = false;
        private boolean indexingComplete = false;
        private long indexStartTime = 0;
        private long actionStartTime = 0;
        private int greenConsecutiveCount = 0;
        private static final int GREEN_CONFIRM_THRESHOLD = 3;
        private static final long STARTUP_DELAY_MS = 100;
        private static final long MAX_SCAN_TIME_MS = 5000;
        private static final long INDEX_TIMEOUT_MS = 2000;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (actionStartTime == 0)
                actionStartTime = System.currentTimeMillis();
            long elapsed = System.currentTimeMillis() - actionStartTime;

            if (detectedPattern == TagConfiguration.RandomizationPattern.PURPLE_PURPLE_GREEN ||
                    detectedPattern == TagConfiguration.RandomizationPattern.UNKNOWN ||
                    indexingComplete) {
                return false;
            }

            if (elapsed < STARTUP_DELAY_MS)
                return true;

            if (!greenConfirmed) {
                if ((elapsed - STARTUP_DELAY_MS) > MAX_SCAN_TIME_MS) {
                    indexingComplete = true;
                    return false;
                }
                RevolverSubsystem.SlotColor color = revolver.readColorNow();
                if (color == RevolverSubsystem.SlotColor.GREEN) {
                    greenConsecutiveCount++;
                    if (greenConsecutiveCount >= GREEN_CONFIRM_THRESHOLD) {
                        greenConfirmed = true;
                        int slots = getSlotsForPattern();
                        if (slots > 0) {
                            revolver.moveIndexerSlots(slots);
                            indexingStarted = true;
                            indexStartTime = System.currentTimeMillis();
                        } else {
                            indexingComplete = true;
                            return false;
                        }
                    }
                } else {
                    greenConsecutiveCount = 0;
                }
                return true;
            }

            if (indexingStarted && !indexingComplete) {
                long indexElapsed = System.currentTimeMillis() - indexStartTime;
                if (revolver.isIndexerAtTarget() || indexElapsed > INDEX_TIMEOUT_MS) {
                    indexingComplete = true;
                    return false;
                }
                return true;
            }
            return !indexingComplete;
        }

        private int getSlotsForPattern() {
            switch (detectedPattern) {
                case GREEN_PURPLE_PURPLE:
                    return 2;
                case PURPLE_GREEN_PURPLE:
                    return 1;
                default:
                    return 0;
            }
        }
    }

    private class SmartIntakeAction implements Action {
        private static final int STATE_MOVING_FORWARD = 0;
        private static final int STATE_PAUSED_FOR_INDEX = 1;
        private static final int STATE_RETURNING = 2;
        private static final int STATE_DONE = 3;

        private final int targetBalls;

        // === CUSTOMIZED COORDINATES FOR RED 2 ===
        // Red Side: Y increases from 20 to 55
        private final double startY = 15;
        private final double endY = 47;
        // ========================================

        private final double moveSpeed = 0.17; // Matched Blue 2
        private final double targetHeading = Math.toRadians(90); // Facing UP (+Y)
        private final double headingKp = 2.0;
        private final double strafeKp = 0.15;

        private int state = STATE_MOVING_FORWARD;
        private int ballsCollected = 0;
        private long indexStartTime = 0;
        private long actionStartTime = 0;
        private double targetX = 0;
        private boolean initialized = false;
        private RevolverSubsystem.SlotColor lastDetected = RevolverSubsystem.SlotColor.EMPTY;

        public SmartIntakeAction(int targetBalls) {
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
                stopDrive();
                return false;
            }

            Pose2d currentPose = drive.localizer.getPose();
            double currentY = currentPose.position.y;
            double currentX = currentPose.position.x;

            double headingError = AngleUnit.normalizeRadians(targetHeading - currentPose.heading.toDouble());
            double xError = targetX - currentX;
            double turnPower = headingError * headingKp;
            double strafeCorrection = -xError * strafeKp; // Negated for Red Alliance (90 deg heading)

            RevolverSubsystem.SlotColor detected = revolver.readColorNow();
            boolean ballDetected = (detected == RevolverSubsystem.SlotColor.GREEN ||
                    detected == RevolverSubsystem.SlotColor.PURPLE);

            packet.put("State", state);
            packet.put("Y", currentY);

            switch (state) {
                case STATE_MOVING_FORWARD:
                    revolver.setIntakePowerDirect(INTAKE_POWER);
                    if (ballDetected && lastDetected == RevolverSubsystem.SlotColor.EMPTY) {
                        stopDrive();
                        revolver.indexerNextSlot();
                        ballsCollected++;
                        Red2SmartAuto.this.collectedBalls++;
                        state = STATE_PAUSED_FOR_INDEX;
                        indexStartTime = System.currentTimeMillis();
                    } else if (currentY >= endY - 1) { // Reach 55 (moving Positive Y)
                        stopDrive();
                        state = STATE_RETURNING;
                    } else if (ballsCollected >= targetBalls) {
                        stopDrive();
                        state = STATE_RETURNING;
                    } else {
                        // Move Forward
                        // Heading 90 (+Y). Forward is +X robot.
                        // So Power X = moveSpeed.
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(moveSpeed, strafeCorrection), turnPower));
                    }
                    break;

                case STATE_PAUSED_FOR_INDEX:
                    revolver.setIntakePowerDirect(INTAKE_POWER);
                    if (revolver.isIndexerAtTarget()
                            || (System.currentTimeMillis() - indexStartTime) >= INTAKE_PAUSE_MS) {
                        lastDetected = RevolverSubsystem.SlotColor.EMPTY;
                        state = STATE_MOVING_FORWARD;
                    }
                    break;

                case STATE_RETURNING:
                    revolver.setIntakePowerDirect(0);
                    // Return to startY (20)
                    // We are at ~55, going to 20. Negative Y.
                    // Heading 90. Forward is +Y.
                    // So we need Negative X (Backup).
                    // dist = (currentY - startY) (positive)

                    if (currentY <= startY + 1) { // Reach 20 (moving Negative Y)
                        stopDrive();
                        state = STATE_DONE;
                    } else {
                        // Move Backwards
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-0.3, strafeCorrection), turnPower));
                    }
                    break;

                case STATE_DONE:
                    stopDrive();
                    revolver.setIntakePowerDirect(0);
                    return false;
            }

            if (!ballDetected)
                lastDetected = RevolverSubsystem.SlotColor.EMPTY;
            else
                lastDetected = detected;

            return true;
        }

        private void stopDrive() {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        }
    }
}
