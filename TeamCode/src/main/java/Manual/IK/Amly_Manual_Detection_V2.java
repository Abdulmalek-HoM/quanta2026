package Manual.IK;

import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;

import java.util.List;

@TeleOp(name = "Amly Manula Detection V2")

public class Amly_Manual_Detection_V2 extends LinearOpMode{

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






    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
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
                new Scalar( 50,   100, 140),
                new Scalar(150, 130, 200)
        );
        //////// The foll0wing lines are for color detection portal ///////
        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(Custome_Blue)         // use a predefined color match
                .setTargetColorRange(ColorRange.BLUE)
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

        /////// End of color detection portal initialization//////


        waitForStart();
        if (opModeIsActive()) {
            imu_IMU.resetYaw();
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                _7ByawPitchRollAnglesVariable_7D = imu_IMU.getRobotYawPitchRollAngles();
                DriveTrain();
                setPower();
                // Pressing Right Bumper will Reset the IMU for Field Centric
                if (gamepad1.options) {
                    imu_IMU.resetYaw();
                }
                if (gamepad1.left_bumper) {
                    multiplier=0.25;

                } else {
                    multiplier=0.6;
                }
                data();
                if (gamepad1.dpad_right) {
                    gripperR.setPosition(Gripper_Close);
//                    sleep(300);
                }
                if (gamepad1.dpad_left) {
                    gripperR.setPosition(0.2);
//                    sleep(300);
                }

                if (gamepad1.dpad_up) {
                    Specimen_Outake();
                    Gripper=0;
//                    sleep(500);
                }
                if (gamepad1.dpad_down) {
                    Specimen_Intake();
                    Gripper=0;
//                    sleep(500);
                }
                if (gamepad1.circle && Slides2 == 0) {
                    // Sample High
                    Sample_High();
                    Slides2 = 1;
                    Gripper=0;
//                    sleep(500);

                }
                if (gamepad1.circle && Slides2 == 1) {
                    Home_Position();
                    // Home Position
                    Slides2 = 0;
                    Gripper=0;
//                    sleep(500);
                }
                if (gamepad1.left_stick_button) {
                    Home_Position();
                    Sample_Intake2 = 0;
                    Slides2 = 0;
                    Gripper =0;
//                    sleep(500);
                }
                if (gamepad1.right_trigger > 0.1 && gamepad1.left_trigger <= 0.1) {
                    ARM12 = ARM12 + 10;
                    telemetry.addData("GRP", ARM12);
                }
                if (gamepad1.left_trigger > 0.1 && gamepad1.right_trigger <= 0.1) {
                    ARM12 = ARM12 - 10;
                    telemetry.addData("GRP1", ARM12);
                }

                if (gamepad1.triangle && Sample_Intake2 == 0) {
                    Sample_Intake();
                    sleep(200);
                    Sample_Intake2 = 1;
                    Gripper=0;
                }
                if (gamepad1.triangle && Sample_Intake2 == 1) {
                    Ground_Grab();
                    sleep(200);
                    Sample_Intake2 = 0;
                    Gripper=0;
                }
                if (gamepad1.touchpad && Gripper==0){
                    gripperL.setPosition(0.5);
                    Gripper=1;
                    sleep(1000);

                }
                if (gamepad1.touchpad && Gripper==1){
                    gripperL.setPosition(0.3);
                    Gripper=2;
                    sleep(1000);

                }
                if (gamepad1.touchpad && Gripper==2){
                    gripperL.setPosition(1);
                    Gripper=0;
                    sleep(1000);


                }


//                detection();

                List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
                ColorBlobLocatorProcessor.Util.filterByArea(700, 8000, blobs);  // filter out very small blobs.

                org.opencv.core.Size myBoxFitSize;
                for (ColorBlobLocatorProcessor.Blob b : blobs) {
                    RotatedRect boxFit = b.getBoxFit();
                    myBoxFitSize = boxFit.size;
                    //AREA = b.getContour().size().area();
                    AREA = myBoxFitSize.area();
                    WIDTH = myBoxFitSize.width;
                    HEIGHT = myBoxFitSize.height;
                    CENTER_X = (int) boxFit.center.x;
                    CENTER_Y = (int) boxFit.center.y;
                    ANGLE = boxFit.angle;


                    // If statements to create the logic of selecting the Correct Oriented Sample
                    CONTOUR_AREA = 8000 > AREA && AREA > 700;
                    CONTOUR_WIDTH = 99 > WIDTH && WIDTH > 40;
                    CONTOUR_HEIGHT = 80 > HEIGHT && HEIGHT > 30;
                    CONTOUR_CENTER = 290 > CENTER_X && CENTER_X > 200 && 230 > CENTER_Y && CENTER_Y > 200;
                    CONTOUR_ANGLE = 100 > ANGLE && ANGLE > 40;
                }

                if (CONTOUR_AREA && CONTOUR_WIDTH && CONTOUR_HEIGHT && CONTOUR_CENTER && CONTOUR_ANGLE){

//                leftFront.setPower(0);
//                rightFront.setPower(0);
//                leftBack.setPower(0);
//                rightBack.setPower(0);
                gamepad1.rumble(500);

                    for (int i = 0; i < 2; i++) {
                        CONTOUR_AREA = false;
                        CONTOUR_WIDTH = false;
                        CONTOUR_HEIGHT = false;
                        CONTOUR_CENTER = false;
                        CONTOUR_ANGLE = false;
                    }




                }
            if (gamepad1.cross && CONTOUR_AREA && CONTOUR_WIDTH && CONTOUR_HEIGHT && CONTOUR_CENTER && CONTOUR_ANGLE) {

                Sample_Intake();
                sleep(1000);
                Ground_Grab();
                for (int i = 0; i < 2; i++) {
                    CONTOUR_AREA = false;
                    CONTOUR_WIDTH = false;
                    CONTOUR_HEIGHT = false;
                    CONTOUR_CENTER = false;
                    CONTOUR_ANGLE = false;
                }
                Gripper=0;



            }

            }
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

    /**
     * Describe this function...
     */
    private void DriveTrain() {
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
        tilting.setPosition(0.45);
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
        ArmBase(500, 1);
//        sleep(500);

        tilting.setPosition(0.60);
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

    private void data() {

        telemetry.addData("IMU Yaw right now:", _7ByawPitchRollAnglesVariable_7D.getYaw(AngleUnit.DEGREES));
        telemetry.addData("rotX", rotX);
        telemetry.addData("rotY", rotY);
        telemetry.addData("Y", y);
        telemetry.addData("X", x);
        telemetry.addData("Theta at the moment (Radians)", _7ByawPitchRollAnglesVariable_7D.getYaw(AngleUnit.RADIANS));
        telemetry.addData("rx", rx);
        telemetry.addData("Multiplier (Speed)", multiplier);
        telemetry.addData("Gripper Position", Gripper);

        telemetry.addData("BaseL Ticks", BaseL.getCurrentPosition());
        telemetry.addData("BaseR Ticks", BaseR.getCurrentPosition());
        telemetry.addData("BaseL Current", ((DcMotorEx) BaseL).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("BaseR Current", ((DcMotorEx) BaseR).getCurrent(CurrentUnit.AMPS));

        telemetry.addData("SlideL Ticks", SlideL.getCurrentPosition());
        telemetry.addData("SlideR Ticks", SlideR.getCurrentPosition());
        telemetry.addData("SlideL Current", ((DcMotorEx) SlideL).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("SlideR Current", ((DcMotorEx) SlideR).getCurrent(CurrentUnit.AMPS));
//        detection();
        //////// testing telemetry with color angle detection
        telemetry.addData("CONTOUR_AREA", CONTOUR_AREA);
        telemetry.addData("CONTOUR_WIDTH", CONTOUR_WIDTH);
        telemetry.addData("CONTOUR_HEIGHT", CONTOUR_HEIGHT);
        telemetry.addData("CONTOUR_CENTER", CONTOUR_CENTER);
        telemetry.addData("CONTOUR_ANGLE", CONTOUR_ANGLE);

        // end of color telemetry
        telemetry.update();
    }

//    private void detection() {
//        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
//                .setTargetColorRange(ColorRange.YELLOW)         // use a predefined color match
//                //.setTargetColorRange(ColorRange.RED)
//                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
//                //.setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))  // search central 1/4 of camera view
//                .setRoi(ImageRegion.asImageCoordinates(210, 145,  285, 220))
//
//                .setDrawContours(true)                        // Show contours on the Stream Preview
//                .setBlurSize(3)                               // Smooth the transitions between different colors in image
//                .setErodeSize(3) // For Low Quallity 2 to 4
//                .setDilateSize(4) // For Low Quallity 2 to 4
//
//                .build();
//
//        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
//        ColorBlobLocatorProcessor.Util.filterByArea(2000, 5000, blobs);  // filter out very small blobs.
//
//        org.opencv.core.Size myBoxFitSize;
//        for (ColorBlobLocatorProcessor.Blob b : blobs) {
//            RotatedRect boxFit = b.getBoxFit();
//            myBoxFitSize = boxFit.size;
//            //AREA = b.getContour().size().area();
//            AREA = myBoxFitSize.area();
//            WIDTH = myBoxFitSize.width;
//            HEIGHT = myBoxFitSize.height;
//            CENTER_X = (int) boxFit.center.x;
//            CENTER_Y = (int) boxFit.center.y;
//            ANGLE = boxFit.angle;
//
//
//            // If statements to create the logic of selecting the Correct Oriented Sample
//            CONTOUR_AREA = 5000 > AREA && AREA > 2000;
//            CONTOUR_WIDTH = 90 > WIDTH && WIDTH > 40;
//            CONTOUR_HEIGHT = 80 > HEIGHT && HEIGHT > 30;
//            CONTOUR_CENTER = 290 > CENTER_X && CENTER_X > 200 && 210 > CENTER_Y && CENTER_Y > 150;
//            CONTOUR_ANGLE = 100 > ANGLE && ANGLE > 40;
//        }
////                           .setRoi(ImageRegion.asImageCoordinates(200, 50,  300, 280))
//
//        }
    }

