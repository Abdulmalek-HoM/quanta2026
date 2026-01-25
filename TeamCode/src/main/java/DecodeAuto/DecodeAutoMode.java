package DecodeAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Decode Auto Logic", group = "DecodeAuto")
public class DecodeAutoMode extends LinearOpMode {

    // Hardware
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    // Tools
    private AprilTagNavigator tagNavigator;
    private AutoState currentState = AutoState.DETECTING_RANDOMIZATION;
    private TagConfiguration.RandomizationPattern detectedPattern = TagConfiguration.RandomizationPattern.UNKNOWN;
    private ElapsedTime stateTimer = new ElapsedTime();

    // Configuration
    private boolean isRedAlliance = true; // Setup mechanism to select this typically needed

    @Override
    public void runOpMode() {
        // Init Hardware (Copied setup from A1/Omni sample)
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Init AprilTag
        tagNavigator = new AprilTagNavigator(hardwareMap, "webCam1");

        telemetry.addData("Status", "Initialized. Waiting for Start.");
        telemetry.update();

        waitForStart();
        stateTimer.reset();

        while (opModeIsActive()) {
            // Main FSM
            switch (currentState) {
                case DETECTING_RANDOMIZATION:
                    detectedPattern = tagNavigator.detectRandomizationPattern();
                    telemetry.addData("State", "Detecting...");
                    telemetry.addData("Pattern", detectedPattern);

                    // Simple logic: if found or timeout (e.g. 5 sec), move on
                    if (detectedPattern != TagConfiguration.RandomizationPattern.UNKNOWN
                            || stateTimer.seconds() > 5.0) {
                        // Decide next step based on pattern
                        telemetry.addData("Locked Pattern", detectedPattern);
                        telemetry.update();

                        // Transition to throwing
                        currentState = AutoState.THROWING_ARTIFACTS;
                        stateTimer.reset();
                    }
                    moveRobot(0, 0, 0); // Stay still
                    break;

                case THROWING_ARTIFACTS:
                    // Here you would implement the "Throwing" logic (e.g. arm movement)
                    // For now, we wait 2 seconds as a placeholder
                    telemetry.addData("State", "Throwing Artifacts for " + detectedPattern);

                    if (stateTimer.seconds() > 2.0) {
                        currentState = AutoState.ALIGNING_FOR_SHOOTING;
                    }
                    moveRobot(0, 0, 0);
                    break;

                case ALIGNING_FOR_SHOOTING:
                    // Determine goal tag based on Alliance (Hardcoded Red for now)
                    int goalTagId = isRedAlliance ? TagConfiguration.ID_RED_SHOOTING_GOAL
                            : TagConfiguration.ID_BLUE_SHOOTING_GOAL;

                    // Use Navigator to get drive powers
                    double[] powers = tagNavigator.getAlignmentPowers(goalTagId);

                    if (powers != null) {
                        double drive = powers[0];
                        double strafe = powers[1];
                        double turn = powers[2];
                        moveRobot(drive, strafe, turn);

                        telemetry.addData("State", "Aligning to Tag " + goalTagId);
                        telemetry.addData("Target", tagNavigator.getDesiredTag().ftcPose.range);

                        // Check if "close enough" / "aligned enough" to shoot
                        // This is a simple check, could be more robust
                        if (Math.abs(tagNavigator.getDesiredTag().ftcPose.range - 12.0) < 1.0 && // 12 inches desired
                                Math.abs(tagNavigator.getDesiredTag().ftcPose.bearing) < 2.0) { // within 2 degrees
                            currentState = AutoState.SHOOTING;
                            stateTimer.reset();
                            moveRobot(0, 0, 0);
                        }
                    } else {
                        // Tag not seen? maybe rotate to find it or stop
                        telemetry.addData("State", "Searching for Tag " + goalTagId);
                        moveRobot(0, 0, 0.1); // Slow scan
                    }
                    break;

                case SHOOTING:
                    telemetry.addData("State", "Shooting!");
                    // Trigger shooter logic
                    if (stateTimer.seconds() > 1.5) {
                        currentState = AutoState.LOCALIZING;
                    }
                    moveRobot(0, 0, 0);
                    break;

                case LOCALIZING:
                    telemetry.addData("State", "Localizing / Parking");
                    // Utilize Localization Tags logic if needed
                    // For now, park/stop
                    moveRobot(0, 0, 0);
                    currentState = AutoState.IDLE;
                    break;

                case IDLE:
                default:
                    moveRobot(0, 0, 0);
                    telemetry.addData("State", "Idle/Finished");
                    break;
            }

            telemetry.update();
        }

        tagNavigator.stop();
    }

    // Helper for basic omni drive
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

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
}
