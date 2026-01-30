package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.RevolverSubsystem;
import DecodeAuto.AprilTagNavigator;
import DecodeAuto.TagConfiguration;

/**
 * A4 - Competition TeleOp with Smart Auto-Indexing + AprilTag Driver Assistance
 * 
 * Combines the best of A3 and A2:
 * - Smart auto-indexing from A3 (automatic ball detection and indexing)
 * - AprilTag alignment assistance from A2 (press Y to auto-align to goal)
 * - Alliance selection support (Red/Blue goal targeting)
 * 
 * Controls:
 * - Y (Triangle): Hold for AprilTag auto-alignment to shooting goal
 * - Share: Toggle alliance (Red ↔ Blue) for correct goal targeting
 * - RT: Intake (auto-indexes when ball detected)
 * - LT: Reverse intake
 * - Circle: Toggle shooter on/off
 * - LB: Kick ball
 * - Cross: Manual index next slot
 * - D-pad Up/Down: Manual indexer trim (±5 ticks)
 * - RB: Slow drive mode
 * - Options: Reset IMU heading
 */
@TeleOp(name = "A4 - Competition (AprilTag Assist)", group = "Competition")
public class A4 extends OpMode {

    // =========================================================================
    // CONFIGURABLE MOTOR POWER SETTINGS
    // =========================================================================
    public static double SHOOTER_POWER = 0.7;
    public static double INTAKE_POWER = 1.0;
    public static double DRIVE_SPEED_NORMAL = 0.7;
    public static double DRIVE_SPEED_SLOW = 0.3;
    // =========================================================================

    // Subsystems
    private RevolverSubsystem revolver;
    private AprilTagNavigator tagNavigator;

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
    private static final double KICKER_RETRACT = 0.3;
    private static final double KICKER_EJECT = 0.8;

    // State - Auto-Indexing
    private RevolverSubsystem.SlotColor lastDetectedColor = RevolverSubsystem.SlotColor.EMPTY;
    private long lastIndexTime = 0;
    private static final long INDEX_COOLDOWN_MS = 800;

    // State - AprilTag Auto-Alignment
    private boolean isAutoAligning = false;
    private boolean isRedAlliance = true; // Default to Red, toggle with Share button
    private boolean lastShare = false;
    private boolean lastLeftStickButton = false;
    private int targetGoalId = TagConfiguration.ID_RED_SHOOTING_GOAL;

    // Drivetrain Multiplier
    private double multiplier = DRIVE_SPEED_NORMAL;

    @Override
    public void init() {
        // 1. Initialize Revolver (RevolverSubsystem for smart indexing)
        revolver = new RevolverSubsystem(hardwareMap);

        // SYNC FSM CONFIG WITH OPMODE CONSTANTS
        revolver.fsmShooterPower = SHOOTER_POWER;
        revolver.fsmIntakePower = INTAKE_POWER;

        // CRITICAL: Disable FSM kicker control so A4 can control it directly
        revolver.disableFSMKickerControl = true;

        // 2. Initialize AprilTag Navigator
        tagNavigator = new AprilTagNavigator(hardwareMap, "webCam1");

        // 3. Initialize Drivetrain Motors
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

        // 4. Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        telemetry.addData("Status", "A4 Competition Mode Ready");
        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
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
                // AprilTag detected - apply alignment powers
                double drive = autoPowers[0];
                double strafe = autoPowers[1];
                double turn = autoPowers[2];

                moveRobot(drive, strafe, turn);
                telemetry.addData("Auto Align", "TRACKING Tag %d (%s)",
                        targetGoalId, isRedAlliance ? "RED" : "BLUE");
            } else {
                // Tag not visible - stop for safety
                moveRobot(0, 0, 0);
                telemetry.addData("Auto Align", "Tag %d NOT FOUND", targetGoalId);

                // DEBUG: Show what WE DO SEE
                java.util.List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> currentDetections = tagNavigator.aprilTag
                        .getDetections();
                telemetry.addData("Visible Tags", currentDetections.size());
                for (org.firstinspires.ftc.vision.apriltag.AprilTagDetection detection : currentDetections) {
                    if (detection.metadata != null) {
                        telemetry.addData(" - Found", "ID %d (%s)", detection.id, detection.metadata.name);
                    } else {
                        telemetry.addData(" - Found", "ID %d (Unknown) - ftcPose: %s", detection.id,
                                (detection.ftcPose == null ? "NULL" : "VALID"));
                        if (detection.rawPose != null) {
                            telemetry.addData("   > RawPose", "Available (x=%.2f)", detection.rawPose.x);
                        }
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
            double y = -gamepad1.left_stick_y; // Inverted Y
            double rx = gamepad1.right_stick_x;

            YawPitchRollAngles botAngles = imu.getRobotYawPitchRollAngles();
            double botHeading = -botAngles.getYaw(AngleUnit.RADIANS);

            // Field centric transformation
            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            // Denominator to ensure no motor power > 1
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

            // Edge detection: ball just entered (wasn't there before)
            boolean justDetected = ballDetected &&
                    (lastDetectedColor == RevolverSubsystem.SlotColor.EMPTY);

            // Auto-index with cooldown to prevent double-triggering
            long currentTime = System.currentTimeMillis();
            if (justDetected && (currentTime - lastIndexTime > INDEX_COOLDOWN_MS)) {
                revolver.indexerNextSlot();
                lastIndexTime = currentTime;
                telemetry.addLine(">>> AUTO-INDEXED! <<<");
            }

            lastDetectedColor = currentColor;
        } else {
            // Reset detection state when not intaking
            lastDetectedColor = RevolverSubsystem.SlotColor.EMPTY;
        }

        // 2. Manual Index (Press Cross) - Always available as override
        if (gamepad1.cross && !lastCross) {
            revolver.indexerNextSlot();
        }
        lastCross = gamepad1.cross;

        // 3. Shooter Toggle (Press Circle)
        if (gamepad1.circle && !lastCircle) {
            isShooterOn = !isShooterOn;
            if (isShooterOn) {
                revolver.setShooterPowerDirect(SHOOTER_POWER);
            } else {
                revolver.setShooterPowerDirect(0);
            }
        }
        lastCircle = gamepad1.circle;

        // 4. Kick (Press Left Bumper) - Direct control
        if (gamepad1.left_bumper && !lastLeftBumper) {
            isKicking = true;
            kickTimer = System.currentTimeMillis();
        }
        lastLeftBumper = gamepad1.left_bumper;

        // Direct kicker logic
        if (isKicking) {
            long elapsed = System.currentTimeMillis() - kickTimer;
            if (elapsed < 500) {
                // Spool up
                revolver.kicker.setPosition(KICKER_RETRACT);
            } else if (elapsed < 1100) {
                // Eject
                revolver.kicker.setPosition(KICKER_EJECT);
            } else if (elapsed < 1700) {
                // Retract
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
        // TELEMETRY
        // =========================================================================
        telemetry.addData("Mode", "A4 - Competition (AprilTag)");
        telemetry.addData("Camera Mode", AprilTagNavigator.CAMERA_UPSIDE_DOWN ? "UPSIDE DOWN (FLIPPED)" : "NORMAL");
        telemetry.addData("Alliance", isRedAlliance ? "RED (Tag 24)" : "BLUE (Tag 20)");
        telemetry.addData("Auto Align", isAutoAligning ? "ACTIVE (Y held)" : "OFF");
        telemetry.addData("Shooter", (isShooterOn ? "ON" : "OFF") + " (Power: " + SHOOTER_POWER + ")");
        telemetry.addData("Detected Color", lastDetectedColor.toString());
        telemetry.addData("Indexer Pos", revolver.getIndexerPosition());
        telemetry.addLine();
        telemetry.addData("Primary", "Y:AprilTag | Share:Alliance | RT:Auto-Intake");
        telemetry.addData("Secondary", "X:Manual Index | LB:Kick | O:Shooter");
        telemetry.addData("Adjust", "D-pad:±5 ticks | RB:Slow | Options:Reset IMU");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (tagNavigator != null) {
            tagNavigator.stop();
        }
    }

    /**
     * Robot-centric movement for AprilTag alignment
     * 
     * @param x   Forward/backward power
     * @param y   Strafe left/right power
     * @param yaw Rotation power
     */
    private void moveRobot(double x, double y, double yaw) {
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        // Normalize powers to [-1, 1]
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
}
