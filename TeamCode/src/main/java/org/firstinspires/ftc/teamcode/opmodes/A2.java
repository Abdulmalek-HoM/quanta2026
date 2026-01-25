package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.SimpleRevolver;
// Import our new AprilTag tools
import DecodeAuto.AprilTagNavigator;
import DecodeAuto.TagConfiguration;

@TeleOp(name = "A2 - Auto Assist", group = "Test")
public class A2 extends OpMode {

    // Subsystems
    private SimpleRevolver revolver;
    private AprilTagNavigator tagNavigator;

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

    // Auto Assist State
    private boolean isAutoAligning = false;
    private int targetGoalId = TagConfiguration.ID_RED_SHOOTING_GOAL; // Default to Red for Test

    // Drivetrain Multiplier
    private double multiplier = 0.5;

    @Override
    public void init() {
        // 1. Initialize Revolver
        revolver = new SimpleRevolver(hardwareMap);

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

        // 4. Initialize AprilTag Navigator
        // Assuming webcam name is "webCam1" as per Auto template, adjust if needed
        tagNavigator = new AprilTagNavigator(hardwareMap, "webCam1");

        telemetry.addData("Status", "A2 Mode Ready. Press Y for Auto-Align.");
    }

    @Override
    public void loop() {
        // =========================================================================
        // DRIVETRAIN LOGIC
        // =========================================================================

        // Auto Align Button (Hold Y / Triangle)
        if (gamepad1.y) {
            isAutoAligning = true;

            // Get override powers from Navigator
            // Assuming RED Goal for A2 test. Can be toggled if needed.
            double[] autoPowers = tagNavigator.getAlignmentPowers(targetGoalId);

            if (autoPowers != null) {
                double drive = autoPowers[0];
                double strafe = autoPowers[1];
                double turn = autoPowers[2];

                // Direct application of Omni powers (Robot centric for alignment usually works
                // best)
                moveRobot(drive, strafe, turn);
                telemetry.addData("Auto Align", "Tracking Tag %d", targetGoalId);
            } else {
                // Search spin or stop? Stop for safety.
                moveRobot(0, 0, 0);
                telemetry.addData("Auto Align", "Tag %d NOT FOUND", targetGoalId);
            }

        } else {
            // MANUAL DRIVING (Field Centric)
            isAutoAligning = false;

            // Reset Yaw
            if (gamepad1.options) {
                imu.resetYaw();
            }

            // Speed Multiplier
            if (gamepad1.right_bumper) {
                multiplier = 0.3;
            } else {
                multiplier = 0.6;
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
        // REVOLVER LOGIC
        // =========================================================================

        // 1. Intake
        if (gamepad1.right_trigger > 0.1) {
            revolver.setIntakePower(1.0);
        } else if (gamepad1.left_trigger > 0.1) {
            revolver.setIntakePower(-1.0);
        } else {
            revolver.setIntakePower(0);
        }

        // 2. Index
        if (gamepad1.cross && !lastCross) {
            revolver.moveNextSlot();
        }
        lastCross = gamepad1.cross;

        // 3. Shooter Toggle
        if (gamepad1.circle && !lastCircle) {
            isShooterOn = !isShooterOn;
            if (isShooterOn) {
                revolver.setShooterPower(0.8);
            } else {
                revolver.setShooterPower(0);
            }
        }
        lastCircle = gamepad1.circle;

        // 4. Kick
        if (gamepad1.left_bumper && !lastLeftBumper) {
            revolver.kick();
        }
        lastLeftBumper = gamepad1.left_bumper;

        // 5. Manual Trim
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
        telemetry.addData("Mode", "A2 - Auto Assist");
        telemetry.addData("Auto Align", isAutoAligning ? "ACTIVE" : "OFF (Hold Y)");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (tagNavigator != null) {
            tagNavigator.stop();
        }
    }

    public void moveRobot(double x, double y, double yaw) {
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
}
