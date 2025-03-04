package Manual;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;
@TeleOp(name = "kkkkkkmode")
public class kkkkkkkmode extends LinearOpMode {

    private IMU imu_IMU;
    private DcMotor gripperArm;
    private DcMotor SlideL;
    private DcMotor SlideR;
    private DcMotor armBase;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor leftBack;
    private Servo tilting;
    private Servo gripperL;
    private Servo gripperR;

    double multiplier;
    YawPitchRollAngles _7ByawPitchRollAnglesVariable_7D;
    double rotX;
    float x;
    float rx;
    double rotY;
    float y;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        int Slides2;
        int hang;
        int ARM12;

        imu_IMU = hardwareMap.get(IMU.class, "imu");
        gripperArm = hardwareMap.get(DcMotor.class, "gripperArm");
        SlideL = hardwareMap.get(DcMotor.class, "SlideL");
        SlideR = hardwareMap.get(DcMotor.class, "SlideR");
        armBase = hardwareMap.get(DcMotor.class, "armBase");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        tilting = hardwareMap.get(Servo.class, "tilting");
        gripperL = hardwareMap.get(Servo.class, "gripperL");
        gripperR = hardwareMap.get(Servo.class, "gripperR");

        // Initializes the IMU with non-default settings. To use this block,
        // plug one of the "new IMU.Parameters" blocks into the parameters socket.
        // Creates a Parameters object for use with an IMU in a REV Robotics Control Hub or
        // Expansion Hub, specifying the hub's orientation on the robot via the direction that
        // the REV Robotics logo is facing and the direction that the USB ports are facing.
        imu_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        gripperArm.setDirection(DcMotor.Direction.FORWARD);
        gripperArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideL.setDirection(DcMotor.Direction.REVERSE);
        SlideR.setDirection(DcMotor.Direction.FORWARD);
        SlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Reverse one of the drive motors.
        ((DcMotorEx) armBase).setVelocityPIDFCoefficients(40, 2, 0, 25);
        ((DcMotorEx) armBase).setPositionPIDFCoefficients(5);
        ((DcMotorEx) gripperArm).setVelocityPIDFCoefficients(40, 2, 2, 25);
        ((DcMotorEx) gripperArm).setPositionPIDFCoefficients(5);
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tilting.setPosition(0.8);
        gripperL.setPosition(0.5);
        gripperR.setPosition(1);
        multiplier = 0.6;
        Slides2 = 0;
        hang = 1;
        ARM12 = 0;

        //////// The foll0wing lines are for color detection portal ///////
        PredominantColorProcessor colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1))
                .setSwatches(
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.BLACK,
                        PredominantColorProcessor.Swatch.WHITE)
                .build();

        /*
         * Build a vision portal to run the Color Sensor process.
         *
         *  - Add the colorSensor process created above.
         *  - Set the desired video resolution.
         *      Since a high resolution will not improve this process, choose a lower resolution that is
         *      supported by your camera.  This will improve overall performance and reduce latency.
         *  - Choose your video source.  This may be
         *      .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))  .....   for a webcam
         *  or
         *      .setCamera(BuiltinCameraDirection.BACK)    ... for a Phone Camera
         */
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorSensor)
                .setCameraResolution(new Size(1280, 720))
                .setCamera(hardwareMap.get(WebcamName.class, "webCam1"))
                .build();

        telemetry.setMsTransmissionInterval(50);  // Speed up telemetry updates, Just use for debugging.
        while (opModeInInit())
        {
            telemetry.addData("DS preview on/off", "3 dots, Camera Stream\n");

            // Request the most recent color analysis.
            // This will return the closest matching colorSwatch and the predominant RGB color.
            // Note: to take actions based on the detected color, simply use the colorSwatch in a comparison or switch.
            //  eg:
            //      if (result.closestSwatch == PredominantColorProcessor.Swatch.RED) {... some code  ...}
            PredominantColorProcessor.Result result = colorSensor.getAnalysis();

            // Display the Color Sensor result.
            telemetry.addData("Best Match:", result.closestSwatch);
            telemetry.addLine(String.format("R %3d, G %3d, B %3d", Color.red(result.rgb), Color.green(result.rgb), Color.blue(result.rgb)));
            telemetry.update();

            sleep(20);
        }

        /////// End of color detection portal initialization//////


        waitForStart();
        if (opModeIsActive()) {
            imu_IMU.resetYaw();
            //PredominantColorProcessor.Result result = colorSensor.getAnalysis();

            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                _7ByawPitchRollAnglesVariable_7D = imu_IMU.getRobotYawPitchRollAngles();
                gamepad();
                setPower();
                // Pressing Right Bumper will Reset the IMU for Field Centric
                if (gamepad1.options) {
                    imu_IMU.resetYaw();
                }
                data();
                if (gamepad1.dpad_right) {
                    gripperR.setPosition(1);
                    sleep(300);
                }
                if (gamepad1.dpad_left) {
                    gripperR.setPosition(0);
                    sleep(300);
                }
                if (gamepad1.cross) {
                    GripperArm(600 + ARM12, 0.8);
                    sleep(500);
                    tilting.setPosition(0.4);
                    gripperL.setPosition(0.5);
                    gripperR.setPosition(0);
                }
                if (gamepad1.dpad_up) {
                    GripperArm(550 + ARM12, 0.8);
                    sleep(500);
                    tilting.setPosition(0.6);
                    gripperL.setPosition(0.5);
                    gripperR.setPosition(0);
                }
                PredominantColorProcessor.Result result = colorSensor.getAnalysis();

                if (gamepad1.dpad_down && result.closestSwatch == PredominantColorProcessor.Swatch.RED) {
                    GripperArm(600 + ARM12, 0.8);
                    sleep(1000);
                    gripperR.setPosition(1);
                    sleep(800);
                    gripperL.setPosition(0.55);
                    tilting.setPosition(0);
                }
                if (gamepad1.touchpad && Slides2 == 0) {
                    // close gripper
                    Slides(5200, 2500);
                    sleep(500);
                    Slides2 = 1;
                }
                if (gamepad1.touchpad && Slides2 == 1) {
                    // close gripper
                    gripperR.setPosition(1);
                    Slides(10, 3000);
                    sleep(500);
                    Slides2 = 0;
                }
                if (gamepad1.left_stick_button) {
                    GripperArm(200 + ARM12, 0.8);
                    tilting.setPosition(0.3);
                    sleep(500);
                    gripperL.setPosition(0.9);
                    sleep(500);
                }
                if (gamepad1.share && hang == 1) {
                    Slides(3500, 3000);
                    GripperArm(300, 0.8);
                    tilting.setPosition(0);
                    sleep(500);
                    gripperL.setPosition(0.1);
                    sleep(500);
                    hang = 0;
                }
                if (gamepad1.share && hang == 0) {
                    Slides(10, 3000);
                    GripperArm(300 + ARM12, 0.8);
                    tilting.setPosition(0);
                    sleep(500);
                    gripperL.setPosition(0.1);
                    sleep(500);
                    hang = 1;
                }
                if (gamepad1.left_bumper) {
                    Slides(1000, 3000);
                    tilting.setPosition(0.2);
                    gripperL.setPosition(0.5);
                    sleep(2000);
                    GripperArm(600 + ARM12, 0.8);
                }
                if (gamepad1.right_bumper) {
                    Slides(10, 3000);
                    sleep(500);
                    GripperArm(40 + ARM12, 0.8);
                    tilting.setPosition(0.4);
                    gripperL.setPosition(0.5);
                    gripperR.setPosition(0);
                }
                if (gamepad1.right_trigger > 0.1 && gamepad1.left_trigger > 0.1 == false) {
                    ARM12 = ARM12 + 10;
                    telemetry.addData("GRP", ARM12);
                }
                if (gamepad1.left_trigger > 0.1 && gamepad1.right_trigger > 0.1 == false) {
                    ARM12 = ARM12 - 10;
                    telemetry.addData("GRP1", ARM12);
                }

//                if (result.closestSwatch == PredominantColorProcessor.Swatch.RED)
//                {
//
//                }

                telemetry.update();
            }
        }
    }

    /**
     * Describe this function...
     */
    private void gamepad() {
        double theta;

        // Use left stick to drive and right stick to turn
        // You may have to negate the sticks. When you
        // negate a stick, negate all other instances of the stick
        x = gamepad1.left_stick_x;
        y = -gamepad1.left_stick_y;
        rx = gamepad1.right_stick_x;
        theta = -_7ByawPitchRollAnglesVariable_7D.getYaw(AngleUnit.DEGREES);
        // Calculated Values
        rotX = x * Math.cos(theta / 180 * Math.PI) - y * Math.sin(theta / 180 * Math.PI);
        rotY = x * Math.sin(theta / 180 * Math.PI) + y * Math.cos(theta / 180 * Math.PI);
    }

    /**
     * Describe this function...
     */
    private void Slides(int SlidesTicks, int SlidesVelocity) {
        SlideL.setTargetPosition(SlidesTicks);
        SlideR.setTargetPosition(SlidesTicks);
        SlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) SlideL).setVelocity(SlidesVelocity);
        ((DcMotorEx) SlideR).setVelocity(SlidesVelocity);
    }

    /**
     * Describe this function...
     */
    private void setPower() {
        // The Y axis of a joystick ranges from -1 in its topmost position to +1 in its bottommost position.
        // We negate this value so that the topmost position corresponds to maximum forward power.
        leftFront.setPower((rotY + rotX + rx) * multiplier);
        rightFront.setPower(((rotY - rotX) - rx) * multiplier);
        // The Y axis of a joystick ranges from -1 in its topmost position to +1 in its bottommost position.
        // We negate this value so that the topmost position corresponds to maximum forward power.
        leftBack.setPower(((rotY - rotX) + rx) * multiplier);
        rightBack.setPower(((rotY + rotX) - rx) * multiplier);
    }

    /**
     * Describe this function...
     */
    private void GripperArm(int Ticks2, double Power2) {
        gripperArm.setTargetPosition(Ticks2);
        gripperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        gripperArm.setPower(Power2);
    }

    /**
     * Describe this function...
     */
    private void data() {
        telemetry.addData("IMU Yaw:", _7ByawPitchRollAnglesVariable_7D.getYaw(AngleUnit.DEGREES));
        telemetry.addData("rotX", rotX);
        telemetry.addData("rotY", rotY);
        telemetry.addData("Y", y);
        telemetry.addData("X", x);
        telemetry.addData("Theta (Radians)", _7ByawPitchRollAnglesVariable_7D.getYaw(AngleUnit.RADIANS));
        telemetry.addData("rx", rx);
        telemetry.addData("Multiplier (Speed)", multiplier);
        telemetry.addData("armBase Ticks", armBase.getCurrentPosition());
        telemetry.addData("gripperArm Ticks", gripperArm.getCurrentPosition());
        telemetry.addData("gripperArm Current", ((DcMotorEx) gripperArm).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("armBase Current", ((DcMotorEx) armBase).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("SlideL Current", ((DcMotorEx) SlideL).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("SlideR Current", ((DcMotorEx) SlideR).getCurrent(CurrentUnit.AMPS));

        //////// testing telemetry with color detection
        PredominantColorProcessor colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1))
                .setSwatches(
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.BLACK,
                        PredominantColorProcessor.Swatch.WHITE)
                .build();
        PredominantColorProcessor.Result result = colorSensor.getAnalysis();

        telemetry.addData("Best Match:", result.closestSwatch);
        telemetry.addLine(String.format("R %3d, G %3d, B %3d", Color.red(result.rgb), Color.green(result.rgb), Color.blue(result.rgb)));
        // end of color telemetry
        telemetry.update();
    }

    /**
     * Describe this function...
     */
    private void ArmBase(int Ticks, double Power) {
        armBase.setTargetPosition(Ticks);
        armBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armBase.setPower(Power);
    }
}
