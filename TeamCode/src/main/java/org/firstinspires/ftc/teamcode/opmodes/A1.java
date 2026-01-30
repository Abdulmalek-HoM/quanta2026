package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.SimpleRevolver;

@TeleOp(name = "A1", group = "Test")
public class A1 extends OpMode {

    // =========================================================================
    // CONFIGURABLE MOTOR POWER SETTINGS - Adjust these values as needed
    // =========================================================================
    public static double SHOOTER_POWER = 0.7; // Shooter motor power (0.0 to 1.0)
    public static double INTAKE_POWER = 1.0; // Intake motor power (0.0 to 1.0)
    public static double DRIVE_SPEED_NORMAL = 0.7; // Normal drive speed
    public static double DRIVE_SPEED_SLOW = 0.3; // Slow mode drive speed
    // =========================================================================

    // Subsystems
    private SimpleRevolver revolver;

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

    // Drivetrain multiplier (updated from constants)
    private double multiplier = DRIVE_SPEED_NORMAL;

    @Override
    public void init() {
        // 1. Initialize Revolver
        revolver = new SimpleRevolver(hardwareMap);

        // 2. Initialize Drivetrain Motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // Directions (Copied from Amly_Manual_Detection)
        // leftFront: FORWARD, leftBack: FORWARD
        // rightFront: REVERSE, rightBack: REVERSE
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

        telemetry.addData("Status", "A1 Mode Ready");
    }

    @Override
    public void loop() {
        // =========================================================================
        // DRIVETRAIN LOGIC (Field Centric)
        // =========================================================================

        // Reset Yaw (Field Centric)
        if (gamepad1.options) {
            imu.resetYaw();
        }

        // Speed Multiplier (Left Bumper usually Slow Mode in Manual, but here LB is
        // Kick)
        // Manual Detection uses LB for slow mode.
        // SimpleTeleOp uses LB for KICK.
        // Let's use Right Bumper for Slow Mode? Or just keep it fixed?
        // User didn't specify Drivetrain controls explicitly other than "gamesticks".
        // I will use default multiplier 0.8 usually, or 0.5 as per Manual Detection.
        // Let's map Slow Mode to Right Bumper (since LB is Kick).
        if (gamepad1.right_bumper) {
            multiplier = DRIVE_SPEED_SLOW;
        } else {
            multiplier = DRIVE_SPEED_NORMAL;
        }

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y; // Inverted Y
        double rx = gamepad1.right_stick_x;

        YawPitchRollAngles botAngles = imu.getRobotYawPitchRollAngles();
        double botHeading = -botAngles.getYaw(AngleUnit.RADIANS); // Note: Manual uses DEGREES and converts manualy.
                                                                  // Helper uses Radians typically.
        // wait, detection code: theta = -yaw(DEGREES); rotX = x * cos(theta/180*PI)...
        // which is just radians.

        // Let's use standard field centric math
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        // Denominator to ensure no motor power > 1
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        // Amly Manual Detection Distribution:
        // LF = (rotY + rotX + rx)
        // RF = (rotY - rotX - rx)
        // LB = (rotY - rotX + rx)
        // RB = (rotY + rotX - rx)

        leftFront.setPower(((rotY + rotX + rx) / denominator) * multiplier);
        rightFront.setPower(((rotY - rotX - rx) / denominator) * multiplier);
        leftBack.setPower(((rotY - rotX + rx) / denominator) * multiplier);
        rightBack.setPower(((rotY + rotX - rx) / denominator) * multiplier);

        // =========================================================================
        // REVOLVER LOGIC (From SimpleTeleOp)
        // =========================================================================

        // 1. Intake Control (Triggers)
        if (gamepad1.right_trigger > 0.1) {
            revolver.setIntakePower(INTAKE_POWER);
        } else if (gamepad1.left_trigger > 0.1) {
            revolver.setIntakePower(-INTAKE_POWER);
        } else {
            revolver.setIntakePower(0);
        }

        // 2. Index / Load (Press Cross)
        if (gamepad1.cross && !lastCross) {
            revolver.moveNextSlot();
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

        // 4. Kick (Press Left Bumper)
        if (gamepad1.left_bumper && !lastLeftBumper) {
            revolver.kick();
        }
        lastLeftBumper = gamepad1.left_bumper;

        // 5. Manual Trim (D-Pad Up/Down)
        if (gamepad1.dpad_up && !lastUp) {
            revolver.manualAdjust(5);
        }
        if (gamepad1.dpad_down && !lastDown) {
            revolver.manualAdjust(-5);
        }
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;

        revolver.update();

        // Telemetry
        telemetry.addData("Mode", "A1 - Drive + Revolver");
        telemetry.addData("Shooter", (isShooterOn ? "ON" : "OFF") + " (Power: " + SHOOTER_POWER + ")");
        telemetry.addData("Heading", String.format("%.1f deg", Math.toDegrees(botHeading)));
        telemetry.addData("Revolver Tgt", revolver.getTargetPos());
        telemetry.addData("Controls", "RT/LT:Intake, LB:Kick, RB:Slow, O:Shoot, X:Next");
        telemetry.update();
    }
}
