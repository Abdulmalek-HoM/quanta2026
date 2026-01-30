package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.RevolverSubsystem;

@TeleOp(name = "A3 - Smart Indexing", group = "Competition")
public class A3 extends OpMode {

    // =========================================================================
    // CONFIGURABLE MOTOR POWER SETTINGS - Adjust these values as needed
    // =========================================================================
    public static double SHOOTER_POWER = 0.7; // Shooter motor power
    public static double INTAKE_POWER = 1.0; // Intake motor power
    public static double DRIVE_SPEED_NORMAL = 0.7; // Normal drive speed
    public static double DRIVE_SPEED_SLOW = 0.3; // Slow mode drive speed
    // =========================================================================

    // Subsystems
    private RevolverSubsystem revolver;

    // Drivetrain Motors
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    // IMU for Field Centric
    private IMU imu;

    // State
    private boolean isShooterOn = false;
    private boolean lastLeftBumper = false;
    private boolean lastCross = false;
    private boolean lastCircle = false;
    private boolean lastUp = false;
    private boolean lastDown = false;

    // Direct kicker control (bypasses RevolverSubsystem FSM)
    private boolean isKicking = false;
    private long kickTimer = 0;
    private static final double KICKER_RETRACT = 0.3;
    private static final double KICKER_EJECT = 0.8;

    // Auto-indexing state
    private RevolverSubsystem.SlotColor lastDetectedColor = RevolverSubsystem.SlotColor.EMPTY;
    private long lastIndexTime = 0;
    private static final long INDEX_COOLDOWN_MS = 800; // Prevent double-indexing

    // Drivetrain Multiplier
    private double multiplier = DRIVE_SPEED_NORMAL;

    @Override
    public void init() {
        // 1. Initialize Revolver (RevolverSubsystem for smart indexing)
        revolver = new RevolverSubsystem(hardwareMap);

        // SYNC FSM CONFIG WITH OPMODE CONSTANTS
        revolver.fsmShooterPower = SHOOTER_POWER;
        revolver.fsmIntakePower = INTAKE_POWER;

        // CRITICAL: Disable FSM kicker control so A3 can control it directly
        revolver.disableFSMKickerControl = true;

        // 2. Initialize Drivetrain Motors
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

        // 3. Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        telemetry.addData("Status", "A3 Smart Indexing Ready");
        telemetry.addData("Info", "RT=Intake (auto-index), LB=Kick, X=Manual index");
    }

    @Override
    public void loop() {
        // =========================================================================
        // DRIVETRAIN LOGIC (Field Centric)
        // =========================================================================

        // Reset Yaw
        if (gamepad1.options) {
            imu.resetYaw();
        }

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

        // Direct kicker logic (like SimpleRevolver)
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
        telemetry.addData("Mode", "A3 - Smart Auto-Index");
        telemetry.addData("Shooter", (isShooterOn ? "ON" : "OFF") + " (Power: " + SHOOTER_POWER + ")");
        telemetry.addData("Detected", lastDetectedColor.toString());
        telemetry.addData("Indexer Pos", revolver.getIndexerPosition());
        telemetry.addData("Heading", String.format("%.1f°", Math.toDegrees(botHeading)));
        telemetry.addLine();
        telemetry.addData("Controls", "RT:Auto-Intake | X:Manual Index | LB:Kick");
        telemetry.addData("Other", "O:Shooter | D-pad:±5 ticks | RB:Slow");
        telemetry.update();
    }
}
