package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;

import java.util.List;

@Autonomous (name = "Amly Auto V1")

public final class test554 extends LinearOpMode {
    private IMU imu_IMU;

    private DcMotor BaseL;
    private DcMotor BaseR;
    private DcMotor SlideL;
    private DcMotor SlideR;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor leftBack;
    private Servo tilting;
    private Servo gripperL;
    private Servo gripperR;

    double Gripper_Close;
    double multiplier;
    double Gripper;

    YawPitchRollAngles _7ByawPitchRollAnglesVariable_7D;
    double rotX;
    float x;
    float rx;
    double rotY;
    float y;
    double AREA = 0;
    double WIDTH = 0;
    double HEIGHT = 0;
    double CENTER_X = 0;
    double CENTER_Y = 0;
    double ANGLE = 0;

    Boolean CONTOUR_AREA = false;
    Boolean CONTOUR_WIDTH = false;
    Boolean CONTOUR_HEIGHT = false;
    Boolean CONTOUR_CENTER = false;
    Boolean CONTOUR_ANGLE = false;


    @Override
    public void runOpMode() throws InterruptedException {

        int Sample_Intake2;
        int Slides2;
//        int hang;
        int ARM12;

        imu_IMU = hardwareMap.get(IMU.class, "imu");
        BaseL = hardwareMap.get(DcMotor.class, "BaseL");
        BaseR = hardwareMap.get(DcMotor.class, "BaseR");
        SlideL = hardwareMap.get(DcMotor.class, "SlideL");
        SlideR = hardwareMap.get(DcMotor.class, "SlideR");
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
        BaseL.setDirection(DcMotor.Direction.FORWARD);
        BaseL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BaseR.setDirection(DcMotor.Direction.REVERSE);
        BaseR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideL.setDirection(DcMotor.Direction.REVERSE);
        SlideR.setDirection(DcMotor.Direction.FORWARD);
        SlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Reverse one of the drive motors.
        ((DcMotorEx) BaseL).setVelocityPIDFCoefficients(20, 2, 2, 25);
        ((DcMotorEx) BaseL).setPositionPIDFCoefficients(5);
        ((DcMotorEx) BaseR).setVelocityPIDFCoefficients(20, 2, 2, 25);
        ((DcMotorEx) BaseR).setPositionPIDFCoefficients(5);
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
        tilting.setPosition(0);
        gripperL.setPosition(1);
        Gripper_Close = 0.6;

        gripperR.setPosition(Gripper_Close);
        multiplier = 0.7;
        Sample_Intake2 = 0;
        Slides2 = 0;
        //hang = 1;
        ARM12 = 0;
        Gripper=0;
        final ColorRange Custome_Blue = new ColorRange(
                ColorSpace.YCrCb,
                new Scalar( 16,   0, 155),
                new Scalar(100, 100, 255)
        );
        //////// The foll0wing lines are for color detection portal ///////
        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(Custome_Blue)         // use a predefined color match
//                .setTargetColorRange(ColorRange.BLUE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                //.setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))  // search central 1/4 of camera view
                .setRoi(ImageRegion.asImageCoordinates(200, 160,  275, 290))

                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)  // too much blurring                             // Smooth the transitions between different colors in image
                .setErodeSize(5) // For Low Quality 2 to 4
                .setDilateSize(5) // For Low Quality 2 to 4

                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
//                .setCameraResolution(new Size(640, 480))
                .setCameraResolution(new Size(640, 480))

                .setCamera(hardwareMap.get(WebcamName.class, "webCam1"))
                .build();

        telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);


        while (opModeInInit())
        {

            telemetry.addData("preview on/off", "... Camera Stream\n");

            // Read the current list
            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

            ColorBlobLocatorProcessor.Util.filterByArea(700, 8000, blobs);  // filter out very small blobs.
            telemetry.addLine(" Area Density Aspect  Center");

            org.opencv.core.Size myBoxFitSize;
            for(ColorBlobLocatorProcessor.Blob b : blobs) {
                RotatedRect boxFit = b.getBoxFit();
                telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)",
                        b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y));
                myBoxFitSize = boxFit.size;

                telemetry.addData("width ", myBoxFitSize.width);
                telemetry.addData("height ", myBoxFitSize.height);
                telemetry.addData("angle ", boxFit.angle);
                telemetry.addData("area ", myBoxFitSize.area());
            }
            telemetry.update();

            sleep(20);
        }


        waitForStart();


        Pose2d beginPose = new Pose2d(18, -59, Math.toRadians(90));
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            Actions.runBlocking(drive.actionBuilder(beginPose)
                    .strafeTo(new Vector2d(21,-60))
                    .splineToConstantHeading(new Vector2d(45, -4), Math.toRadians(-42.95))
                    .waitSeconds(0.1)
                    .strafeTo(new Vector2d(47,-52))
                            .waitSeconds(0.1)
//                    .splineToConstantHeading(new Vector2d(60, -4), Math.toRadians(-42.95))
//                    .waitSeconds(0)
//                    .strafeTo(new Vector2d(60,-52))
//                    .waitSeconds(0.1)
                    .build());



            Actions.runBlocking(drive.actionBuilder(new Pose2d(47, -52, Math.toRadians(90.00)))
                    //.splineToConstantHeading(new Vector2d(37, -59), Math.toRadians(268.94))
                    .turn(Math.toRadians(180))

                    .strafeTo(new Vector2d(45 ,-40))
                    .build());

            Specimen_Intake();
            sleep(800);

            Actions.runBlocking(drive.actionBuilder(new Pose2d(45 ,-40, Math.toRadians(270)))
                    //.splineToConstantHeading (new Vector2d(37, -59), Math.toRadians(268.94))
                    .strafeTo(new Vector2d(45 ,-53))
                    .build());

            Specimen_Outake();
            sleep(800);

            Actions.runBlocking(drive.actionBuilder(new Pose2d(45, -53, Math.toRadians(270)))
//                    .splineToConstantHeading(new Vector2d(-10, -25), Math.toRadians(270))
                    .strafeTo(new Vector2d(-18 ,-29))
                    .build());
            sleep(800);

            Specimen_Intake();
            sleep(800);

            Actions.runBlocking(drive.actionBuilder(new Pose2d(-18, -29, Math.toRadians(270)))
                    //.splineToConstantHeading(new Vector2d(37, -59), Math.toRadians(268.94))
                    .strafeTo(new Vector2d(45 ,-47))
                    .waitSeconds(0.05)
                    .strafeTo(new Vector2d(45 ,-55))
                    .build());
            sleep(800);

            Specimen_Outake();
            sleep(800);
            //sleep(200);

            Actions.runBlocking(drive.actionBuilder(new Pose2d(45, -55, Math.toRadians(270)))
//                    .splineToConstantHeading(new Vector2d(-13, -25), Math.toRadians(270))
                    .strafeTo(new Vector2d(-15, -30.5))

                    .build());
            sleep(800);

            Specimen_Intake();
            sleep(800);


            Actions.runBlocking(drive.actionBuilder(new Pose2d(-15, -30.5, Math.toRadians(270)))
                    //.splineToConstantHeading(new Vector2d(37, -59), Math.toRadians(268.94))
                    .strafeTo(new Vector2d(50 ,-60))
                    .build());
//            Specimen_Outake();
//            sleep(1000);
//
//            Actions.runBlocking(drive.actionBuilder(new Pose2d(48, -57, Math.toRadians(270)))
////                    .splineToConstantHeading(new Vector2d(-18, -25), Math.toRadians(270))
//                    .strafeTo(new Vector2d(-2, -28))
//
//                    .build());
//
//            Specimen_Intake();
//            sleep(1000);
//
//
//
//            Actions.runBlocking(drive.actionBuilder(new Pose2d(-2, -28, Math.toRadians(270)))
//                    //.splineToConstantHeading(new Vector2d(37, -59), Math.toRadians(268.94))
//                    .strafeTo(new Vector2d(50 ,-57))
//                    .waitSeconds(0.05)
//                    .strafeTo(new Vector2d(50 ,-60))
//                    .build());


//            Actions.runBlocking(drive.actionBuilder(new Pose2d(37, -59, Math.toRadians(270)))
//                    .splineToConstantHeading(new Vector2d(-21, -25), Math.toRadians(270))
//                                .strafeTo(new Vector2d(-21, -25))

//                    .build());







        }
    }
    private void ArmBase(int ArmTicks, double ArmPower) {
        BaseL.setTargetPosition(ArmTicks);
        BaseR.setTargetPosition(ArmTicks);
        BaseL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BaseR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BaseL.setPower(ArmPower);
        BaseR.setPower(ArmPower);
    }

    private void Slides(int SlidesTicks, int SlidesVelocity) {
        SlideL.setTargetPosition(SlidesTicks);
        SlideR.setTargetPosition(SlidesTicks);
        SlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) SlideL).setVelocity(SlidesVelocity);
        ((DcMotorEx) SlideR).setVelocity(SlidesVelocity);
    }


    private void Home_Position() {
        gripperR.setPosition(Gripper_Close);
        gripperL.setPosition(1);
        tilting.setPosition(0.4);

        Slides(0, 3000);
        sleep(1500);
        ArmBase(0, 0.5);
    }

    /**
     * Describe this function...
     */
    private void Sample_Intake() {
        gripperR.setPosition(0.2);
        gripperL.setPosition(1);
        tilting.setPosition(0.55);
        ArmBase(0, 1);
//        sleep(1000);
        Slides(3000, 3000);
    }

    /**
     * Describe this function...
     */
    private void Specimen_Intake() {
        tilting.setPosition(0.85);
        sleep(300);

        ArmBase(0, 1);
        sleep(300);

        tilting.setPosition(0.35);
//        sleep(500);
        gripperR.setPosition(0.2);
        gripperL.setPosition(1);
        Slides(0, 3000);
    }

    /**
     * Describe this function...
     */
    private void Specimen_Outake() {
        gripperR.setPosition(Gripper_Close);
        sleep(500);
        tilting.setPosition(0.60);
        sleep(500);

        ArmBase(515, 1);
        gripperL.setPosition(1);
        Slides(0, 3000);
    }

    /**
     * Describe this function...
     */
    private void Sample_High() {
        gripperR.setPosition(Gripper_Close);
        gripperL.setPosition(1);
        tilting.setPosition(0.5);
        ArmBase(240, 1);
        sleep(500);
        Slides(1200, 3000);
    }

    /**
     * Describe this function...
     */
    private void Ground_Grab() {
        tilting.setPosition(0.8 );
        sleep(500);
        gripperR.setPosition(1);
        sleep(300);
        tilting.setPosition(0.3);
        gripperL.setPosition(1);

//        Home_Position();
    }

}



