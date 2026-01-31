package DecodeAuto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

/**
 * DrivetrainTestAuto - Road Runner 1.0 Autonomous Mode
 * 
 * This OpMode tests the drivetrain using Road Runner action builders.
 * It follows the Decode autonomous strategy waypoints without any mechanism
 * actions.
 * 
 * Waypoints:
 * 1. Start: (-61, -35, 0°)
 * 2. Read AprilTag: spline to (-37, -13, 160°)
 * 3. Shooting Position: spline to (-30, -31, 230°)
 * 4. Intake Position 1: spline to (-11, -30, 270°)
 * 5. Intake Position 2: strafe to (-11, -55, 270°) then back to (-11, -30,
 * 270°)
 * 6. Return to Shooting Position: spline to (-30, -31, 230°)
 * 7. Ramp Gate Position: spline to (-30, -31, 230°)
 */
@Config
@Disabled
@Autonomous(name = "Drivetrain Test Auto v1.0", group = "DecodeAuto")
public class DrivetrainTestAuto extends LinearOpMode {

    // Starting pose
    public static double START_X = -61;
    public static double START_Y = -35;
    public static double START_HEADING_DEG = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the mecanum drive with the starting pose
        Pose2d startPose = new Pose2d(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Build the trajectory sequence using Road Runner 1.0 action builders
        Action trajectorySequence = drive.actionBuilder(startPose)
                // Step 2: Read AprilTag - spline to (-37, -13, 160°)
                .strafeToLinearHeading(new com.acmerobotics.roadrunner.Vector2d(-37, -13), Math.toRadians(160))
                .waitSeconds(1)
                // Step 3: Shooting Position - spline to (-30, -31, 230°)
                .strafeToLinearHeading(new com.acmerobotics.roadrunner.Vector2d(-30, -31), Math.toRadians(230))
                .waitSeconds(1)
                // Step 4: Intake Position 1 - spline to (-11, -30, 270°)
                .strafeToLinearHeading(new com.acmerobotics.roadrunner.Vector2d(-11, -30), Math.toRadians(270))
                .waitSeconds(1)
                // Step 5: Intake Position 2 - strafe to (-11, -55, 270°) then back
                .strafeToLinearHeading(new com.acmerobotics.roadrunner.Vector2d(-11, -55), Math.toRadians(270))
                .strafeToLinearHeading(new com.acmerobotics.roadrunner.Vector2d(-11, -30), Math.toRadians(270))
                .waitSeconds(1)

                // Step 6: Return to Shooting Position - spline to (-30, -31, 230°)
                .strafeToLinearHeading(new com.acmerobotics.roadrunner.Vector2d(-30, -31), Math.toRadians(230))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-5, -53), Math.toRadians(90))

                // Step 7: Ramp Gate Position (same as shooting) - spline to (-30, -31, 230°)
                // Note: Since this is the same position, we'll add a small wait or just hold
                // position
                .waitSeconds(1)

                .build();

        // Telemetry for initialization
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Start Pose", "x=%.1f, y=%.1f, heading=%.1f°", START_X, START_Y, START_HEADING_DEG);
        telemetry.addLine();
        telemetry.addLine("Trajectory Waypoints:");
        telemetry.addData("1. Start", "(-61, -35, 0°)");
        telemetry.addData("2. AprilTag Read", "(-37, -13, 160°)");
        telemetry.addData("3. Shooting", "(-30, -31, 230°)");
        telemetry.addData("4. Intake Pos 1", "(-11, -30, 270°)");
        telemetry.addData("5. Intake Pos 2", "strafe (-11, -55, 270°) → (-11, -30, 270°)");
        telemetry.addData("6. Shooting", "(-30, -31, 230°)");
        telemetry.addData("7. Ramp Gate", "(-30, -31, 230°)");
        telemetry.update();

        waitForStart();

        if (isStopRequested())
            return;

        // Execute the trajectory sequence
        telemetry.addData("Status", "Running trajectory...");
        telemetry.update();

        Actions.runBlocking(trajectorySequence);

        // Done
        telemetry.addData("Status", "Trajectory Complete!");
        telemetry.update();

        // Keep the OpMode running to see final telemetry
        while (opModeIsActive() && !isStopRequested()) {
            sleep(50);
        }
    }
}
