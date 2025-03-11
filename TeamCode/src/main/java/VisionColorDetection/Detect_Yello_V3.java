package VisionColorDetection;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import org.openftc.easyopencv.OpenCvPipeline;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import ColorObjectTracking.CameraFusedPID;

@TeleOp(name = "Detecting Yello Custom V3")
public class Detect_Yello_V3 extends LinearOpMode {
	ElapsedTime timer = new ElapsedTime();
	private double lastError = 0;

	private BNO055IMU imu;
	double cX = 0;
	double cY = 0;
	double width = 0;

	private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
	/** MAKE SURE TO CHANGE THE FOV AND THE RESOLUTIONS ACCORDINGLY **/
	private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
	private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution
	private static final double FOV = 40;

	// Calculate the distance using the formula
	public static final double objectWidthInRealWorldUnits = 1.456693;  // Replace with the actual width of the object in real-world units
	public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels


	@Override
	public void runOpMode() {

		initOpenCV();
		FtcDashboard dashboard = FtcDashboard.getInstance();
		telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
		FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);


		waitForStart();

		while (opModeIsActive()) {


			telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
			telemetry.addData("Distance in Inch", (getDistance(width)));
			telemetry.update();// The OpenCV pipeline automatically processes frames and handles detection
		}

		// Release resources
		controlHubCam.stopStreaming();
	}

	private void initOpenCV() {

		// Create an instance of the camera
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
				"cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

		// Use OpenCvCameraFactory class from FTC SDK to create camera instance
		controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
				hardwareMap.get(WebcamName.class, "webCam1"), cameraMonitorViewId);

		controlHubCam.setPipeline(new Detecting_Yello_Custom_Pipline_V3());

		controlHubCam.openCameraDevice();
		controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
	}



	public class Detecting_Yello_Custom_Pipline_V3 extends OpenCvPipeline {


		public Scalar lowerRGB = new Scalar(179.0, 139.0, 0.0, 0.0);
		public Scalar upperRGB = new Scalar(255.0, 255.0, 120.0, 0.0);
		private Mat rgbMat = new Mat();
		private Mat rgbBinaryMat = new Mat();

		private ArrayList<MatOfPoint> contours = new ArrayList<>();
		private Mat hierarchy = new Mat();

		public int minArea = 500;
		public int maxArea = 5000;
		private ArrayList<MatOfPoint> contoursByArea = new ArrayList<>();

		public Scalar lineColor = new Scalar(0.0, 255.0, 0.0, 0.0);
		public int lineThickness = 3;

		private Mat inputContours = new Mat();

		@Override
		public Mat processFrame(Mat input) {
			Imgproc.cvtColor(input, rgbMat, Imgproc.COLOR_RGBA2RGB);
			Core.inRange(rgbMat, lowerRGB, upperRGB, rgbBinaryMat);

			contours.clear();
			hierarchy.release();
			Imgproc.findContours(rgbBinaryMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

			contoursByArea.clear();
			for (MatOfPoint contour : contours) {
				double area = Imgproc.contourArea(contour);
				if ((area >= minArea) && (area <= maxArea)) {
					contoursByArea.add(contour);
				}
			}
			org.opencv.core.Size myBoxFitSize;
			for (MatOfPoint contour : contours) {
				myBoxFitSize = contour.size();
				telemetry.addData("width", myBoxFitSize.width);
				telemetry.addData("height", myBoxFitSize.height);


			}


			input.copyTo(inputContours);
			Imgproc.drawContours(inputContours, contoursByArea, -1, lineColor, lineThickness);

			return inputContours;
		}
	}
	private static double getAngleTarget(double objMidpoint){
		double midpoint = -((objMidpoint - (CAMERA_WIDTH/2))*FOV)/CAMERA_WIDTH;
		return midpoint;
	}
	private static double getDistance(double width){
		double distance = (objectWidthInRealWorldUnits * focalLength) / width;
		return distance;
	}

	public double angleWrap(double radians){
		while(radians > Math.PI){
			radians -= 2 * Math.PI;
		}
		while(radians < -Math.PI){
			radians += 2 * Math.PI;
		}
		return radians;
	}




}