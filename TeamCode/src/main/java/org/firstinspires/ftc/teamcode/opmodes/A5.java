package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.RevolverSubsystem;
import DecodeAuto.AprilTagNavigator;
import DecodeAuto.TagConfiguration;

/**
 * A5 - Competition TeleOp with VELOCITY CONTROL + PIDF Tuning
 * 
 * Based on A4, with the following changes:
 * - Shooter motor uses VELOCITY control (ticks/sec) instead of raw power
 * - Indexer motor uses custom PIDF coefficients to prevent position drift
 * 
 * All motor parameters are configurable at the top of this file.
 * 
 * Controls:
 * - Y (Triangle): Hold for AprilTag auto-alignment to shooting goal
 * - Share: Toggle alliance (Red ↔ Blue) for correct goal targeting
 * - RT: Intake (auto-indexes when ball detected)
 * - LT: Reverse intake
 * - Circle: Toggle shooter on/off (VELOCITY MODE)
 * - LB: Kick ball
 * - Cross: Manual index next slot
 * - D-pad Up/Down: Manual indexer trim (±5 ticks)
 * - RB: Slow drive mode
 * - Options: Reset IMU heading
 */
@TeleOp(name = "A5 - Velocity + PIDF Control", group = "Competition")
public class A5 extends OpMode {

    // =========================================================================
    // === SHOOTER MOTOR PARAMETERS (VELOCITY CONTROL) ===
    // =========================================================================
    /** Shooter target velocity in ticks per second */
    public static double SHOOTER_VELOCITY_TPS = 2000.0; // max = 3000

    /** Shooter PIDF Coefficients (for RUN_USING_ENCODER mode) */
    public static double SHOOTER_P = 10.0;
    public static double SHOOTER_I = 3.0;
    public static double SHOOTER_D = 0.0;
    public static double SHOOTER_F = 12.0;
    // =========================================================================

    // =========================================================================
    // === INDEXER MOTOR PARAMETERS (POSITION CONTROL WITH PIDF) ===
    // =========================================================================
    /** Indexer PIDF Coefficients to prevent position drift */
    public static double INDEXER_P = 10.0;
    public static double INDEXER_I = 1.0;
    public static double INDEXER_D = 2.0;
    public static double INDEXER_F = 10.0;
    // =========================================================================

    // =========================================================================
    // === OTHER MOTOR POWER SETTINGS ===
    // =========================================================================
    public static double INTAKE_POWER = 1.0;
    public static double DRIVE_SPEED_NORMAL = 1.0;
    public static double DRIVE_SPEED_SLOW = 0.5;
    // =========================================================================

    // Subsystems
    private RevolverSubsystem revolver;
    private AprilTagNavigator tagNavigator;

    // Direct motor references for PIDF configuration
    private DcMotorEx shooterMotor;
    private DcMotorEx indexerMotor;

    // Drivetrain Motors
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    // IMU for Field Centric
    private IMU imu;

    // State - Revolver Controls
    private boolean isShooterOn = false;
    private boolean lastLeftBumper = false;
    private boolean lastCross = false;
    private boolean lastCircle = false;
    private boolean lastUp = false;
    private boolean lastDown = false;

    // State - Direct Kicker Control
    private boolean isKicking = false;
    private long kickTimer = 0;
    private static final double KICKER_RETRACT = 0.2;
    private static final double KICKER_EJECT = 0.8;

    // State - Auto-Indexing
    private RevolverSubsystem.SlotColor lastDetectedColor = RevolverSubsystem.SlotColor.EMPTY;
    private long lastIndexTime = 0;
    private static final long INDEX_COOLDOWN_MS = 800;

    // State - AprilTag Auto-Alignment
    private boolean isAutoAligning = false;
    private boolean isRedAlliance = true;
    private boolean lastShare = false;
    private boolean lastLeftStickButton = false;
    private int targetGoalId = TagConfiguration.ID_RED_SHOOTING_GOAL;

    // Drivetrain Multiplier
    private double multiplier = DRIVE_SPEED_NORMAL;

    // Software PID state for indexer
    private int indexerTargetPosition = 0;
    private double indexerIntegral = 0;
    private double indexerLastError = 0;
    private long lastPIDUpdateTime = 0;

    @Override
    public void init() {
        // 1. Initialize Revolver (RevolverSubsystem for smart indexing)
        revolver = new RevolverSubsystem(hardwareMap);

        // SYNC FSM CONFIG WITH OPMODE CONSTANTS
        revolver.fsmIntakePower = INTAKE_POWER;

        // CRITICAL: Disable FSM kicker control so A5 can control it directly
        revolver.disableFSMKickerControl = true;

        // 2. Get direct references to motors for PIDF configuration
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        indexerMotor = hardwareMap.get(DcMotorEx.class, "indexer");

        // 3. Configure SHOOTER motor for VELOCITY control
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Apply shooter PIDF coefficients
        PIDFCoefficients shooterPIDF = new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);

        telemetry.addData("Shooter PIDF", String.format("P=%.1f I=%.1f D=%.1f F=%.1f",
                (double) SHOOTER_P, (double) SHOOTER_I, (double) SHOOTER_D, (double) SHOOTER_F));

        // 4. Configure INDEXER motor - use RUN_WITHOUT_ENCODER for software PID control
        // NOTE: Hardware PIDF for RUN_TO_POSITION is not supported on all REV Hub
        // motors
        // We will implement software-based position control instead
        indexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        indexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize software PID target
        indexerTargetPosition = 0;

        telemetry.addData("Indexer Mode", "Software PID Control");
        telemetry.addData("Indexer PIDF", String.format("P=%.1f I=%.1f D=%.1f F=%.1f",
                (double) INDEXER_P, (double) INDEXER_I, (double) INDEXER_D, (double) INDEXER_F));

        // 5. Initialize AprilTag Navigator
        tagNavigator = new AprilTagNavigator(hardwareMap, "webCam1");

        // 6. Initialize Drivetrain Motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // Directions
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Zero Power Behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 7. Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        telemetry.addData("Status", "A5 - Velocity + PIDF Control Ready");
        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.addData("Shooter Mode", "VELOCITY (" + SHOOTER_VELOCITY_TPS + " tps)");
        telemetry.addData("Info", "Y=AprilTag Align | Share=Toggle Alliance");
    }

    @Override
    public void loop() {
        // =========================================================================
        // ALLIANCE SELECTION (Share Button)
        // =========================================================================
        if (gamepad1.share && !lastShare) {
            isRedAlliance = !isRedAlliance;
            targetGoalId = isRedAlliance ? TagConfiguration.ID_RED_SHOOTING_GOAL
                    : TagConfiguration.ID_BLUE_SHOOTING_GOAL;

            telemetry.addLine(">>> ALLIANCE CHANGED: " + (isRedAlliance ? "RED" : "BLUE") + " <<<");
        }
        lastShare = gamepad1.share;

        // =========================================================================
        // DRIVETRAIN LOGIC - AprilTag Auto-Align OR Manual Field-Centric
        // =========================================================================

        if (gamepad1.y) {
            // *** APRILTAG AUTO-ALIGNMENT MODE ***
            isAutoAligning = true;

            double[] autoPowers = tagNavigator.getAlignmentPowers(targetGoalId);

            if (autoPowers != null) {
                double drive = autoPowers[0];
                double strafe = autoPowers[1];
                double turn = autoPowers[2];

                moveRobot(drive, strafe, turn);
                telemetry.addData("Auto Align", "TRACKING Tag %d (%s)",
                        targetGoalId, isRedAlliance ? "RED" : "BLUE");
            } else {
                moveRobot(0, 0, 0);
                telemetry.addData("Auto Align", "Tag %d NOT FOUND", targetGoalId);

                // DEBUG: Show detected tags
                java.util.List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> currentDetections = tagNavigator.aprilTag
                        .getDetections();
                telemetry.addData("Visible Tags", currentDetections.size());
                for (org.firstinspires.ftc.vision.apriltag.AprilTagDetection detection : currentDetections) {
                    if (detection.metadata != null) {
                        telemetry.addData(" - Found", "ID %d (%s)", detection.id, detection.metadata.name);
                    } else {
                        telemetry.addData(" - Found", "ID %d (Unknown)", detection.id);
                    }
                }
            }

        } else {
            // *** MANUAL FIELD-CENTRIC DRIVING ***
            isAutoAligning = false;

            // Reset Yaw
            if (gamepad1.options) {
                imu.resetYaw();
            }

            // TOGGLE CAMERA FLIP (Left Stick Button)
            if (gamepad1.left_stick_button && !lastLeftStickButton) {
                AprilTagNavigator.CAMERA_UPSIDE_DOWN = !AprilTagNavigator.CAMERA_UPSIDE_DOWN;
            }
            lastLeftStickButton = gamepad1.left_stick_button;

            // Speed Multiplier (Right Bumper = Slow Mode)
            if (gamepad1.right_bumper) {
                multiplier = DRIVE_SPEED_SLOW;
            } else {
                multiplier = DRIVE_SPEED_NORMAL;
            }

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            YawPitchRollAngles botAngles = imu.getRobotYawPitchRollAngles();
            double botHeading = -botAngles.getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            leftFront.setPower(((rotY + rotX + rx) / denominator) * multiplier);
            rightFront.setPower(((rotY - rotX - rx) / denominator) * multiplier);
            leftBack.setPower(((rotY - rotX + rx) / denominator) * multiplier);
            rightBack.setPower(((rotY + rotX - rx) / denominator) * multiplier);
        }

        // =========================================================================
        // REVOLVER LOGIC WITH SMART AUTO-INDEXING
        // =========================================================================

        // 1. Intake Control + Auto-Indexing
        boolean intaking = false;
        if (gamepad1.right_trigger > 0.1) {
            revolver.setIntakePowerDirect(INTAKE_POWER);
            intaking = true;
        } else if (gamepad1.left_trigger > 0.1) {
            revolver.setIntakePowerDirect(-INTAKE_POWER);
        } else {
            revolver.setIntakePowerDirect(0);
        }

        // AUTO-INDEXING: Detect ball during intake and rotate automatically
        if (intaking) {
            RevolverSubsystem.SlotColor currentColor = revolver.readColorNow();
            boolean ballDetected = (currentColor == RevolverSubsystem.SlotColor.GREEN ||
                    currentColor == RevolverSubsystem.SlotColor.PURPLE);

            boolean justDetected = ballDetected &&
                    (lastDetectedColor == RevolverSubsystem.SlotColor.EMPTY);

            long currentTime = System.currentTimeMillis();
            if (justDetected && (currentTime - lastIndexTime > INDEX_COOLDOWN_MS)) {
                revolver.indexerNextSlot();
                lastIndexTime = currentTime;
                telemetry.addLine(">>> AUTO-INDEXED! <<<");
            }

            lastDetectedColor = currentColor;
        } else {
            lastDetectedColor = RevolverSubsystem.SlotColor.EMPTY;
        }

        // 2. Manual Index (Press Cross)
        if (gamepad1.cross && !lastCross) {
            revolver.indexerNextSlot();
        }
        lastCross = gamepad1.cross;

        // 3. Shooter Toggle (Press Circle) - VELOCITY MODE
        if (gamepad1.circle && !lastCircle) {
            isShooterOn = !isShooterOn;
            if (isShooterOn) {
                // Set velocity instead of raw power
                shooterMotor.setVelocity(SHOOTER_VELOCITY_TPS);
            } else {
                shooterMotor.setVelocity(0);
            }
        }
        lastCircle = gamepad1.circle;

        // 4. Kick (Press Left Bumper)
        if (gamepad1.left_bumper && !lastLeftBumper) {
            isKicking = true;
            kickTimer = System.currentTimeMillis();
        }
        lastLeftBumper = gamepad1.left_bumper;

        if (isKicking) {
            long elapsed = System.currentTimeMillis() - kickTimer;
            if (elapsed < 500) {
                revolver.kicker.setPosition(KICKER_RETRACT);
            } else if (elapsed < 1100) {
                revolver.kicker.setPosition(KICKER_EJECT);
            } else if (elapsed < 1700) {
                revolver.kicker.setPosition(KICKER_RETRACT);
            } else {
                isKicking = false;
            }
        }

        // 5. Manual Trim (D-Pad Up/Down)
        if (gamepad1.dpad_up && !lastUp) {
            revolver.manualAdjust(5);
        }
        if (gamepad1.dpad_down && !lastDown) {
            revolver.manualAdjust(-5);
        }
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;

        // Update subsystem
        revolver.update();

        // =========================================================================
        // SOFTWARE PID CONTROL FOR INDEXER (bypasses RevolverSubsystem)
        // =========================================================================
        updateIndexerPID();

        // =========================================================================
        // TELEMETRY - Enhanced for Velocity + PIDF Monitoring
        // =========================================================================
        telemetry.addData("Mode", "A5 - Velocity + PIDF");
        telemetry.addData("Camera Mode", AprilTagNavigator.CAMERA_UPSIDE_DOWN ? "UPSIDE DOWN" : "NORMAL");
        telemetry.addData("Alliance", isRedAlliance ? "RED (Tag 24)" : "BLUE (Tag 20)");
        telemetry.addData("Auto Align", isAutoAligning ? "ACTIVE (Y held)" : "OFF");
        telemetry.addLine();

        // Shooter status with velocity info
        double currentShooterVelocity = shooterMotor.getVelocity();
        telemetry.addData("Shooter", isShooterOn ? "ON" : "OFF");
        telemetry.addData("  Target Vel", String.format("%.0f tps", (double) SHOOTER_VELOCITY_TPS));
        telemetry.addData("  Actual Vel", String.format("%.0f tps", (double) currentShooterVelocity));

        double errorPct = (isShooterOn && SHOOTER_VELOCITY_TPS > 0)
                ? ((SHOOTER_VELOCITY_TPS - currentShooterVelocity) / SHOOTER_VELOCITY_TPS * 100.0)
                : 0.0;
        telemetry.addData("  Error", String.format("%.1f%%", errorPct));

        // Indexer status with position info
        telemetry.addLine();
        telemetry.addData("Indexer Pos", String.format("%.3f", (double) revolver.getIndexerPosition()));
        telemetry.addData("  Target", indexerMotor.getTargetPosition());
        telemetry.addData("  Current", indexerMotor.getCurrentPosition());
        telemetry.addData("  At Target", revolver.isIndexerAtTarget() ? "YES" : "no");

        telemetry.addLine();
        telemetry.addData("Detected Color", lastDetectedColor.toString());
        telemetry.addLine();
        telemetry.addData("Controls", "Y:AprilTag | Share:Alliance | O:Shooter");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (tagNavigator != null) {
            tagNavigator.stop();
        }
        shooterMotor.setVelocity(0);
    }

    /**
     * Robot-centric movement for AprilTag alignment
     */
    private void moveRobot(double x, double y, double yaw) {
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }

    /**
     * Software PID controller for indexer position
     * Called every loop() to maintain position
     */
    private void updateIndexerPID() {
        // Get current time for delta calculation
        long currentTime = System.currentTimeMillis();
        if (lastPIDUpdateTime == 0) {
            lastPIDUpdateTime = currentTime;
            return;
        }
        double dt = (currentTime - lastPIDUpdateTime) / 1000.0; // seconds
        lastPIDUpdateTime = currentTime;

        // Sync target from RevolverSubsystem using public getter
        indexerTargetPosition = (int) revolver.getIndexerTargetPos();

        // Calculate error
        int currentPos = indexerMotor.getCurrentPosition();
        double error = indexerTargetPosition - currentPos;

        // PID calculations
        indexerIntegral += error * dt;
        // Anti-windup: clamp integral
        indexerIntegral = Math.max(-1000, Math.min(1000, indexerIntegral));

        double derivative = (error - indexerLastError) / dt;
        indexerLastError = error;

        // Calculate power output
        double power = (INDEXER_P * error / 1000.0) +
                (INDEXER_I * indexerIntegral / 1000.0) +
                (INDEXER_D * derivative / 1000.0);

        // Clamp power to [-1, 1]
        power = Math.max(-1.0, Math.min(1.0, power));

        // Apply power to motor
        indexerMotor.setPower(power);
    }

    /**
     * Set new indexer target position for software PID
     */
    private void setIndexerTarget(int targetTicks) {
        indexerTargetPosition = targetTicks;
        indexerIntegral = 0; // Reset integral on new target
    }
}
