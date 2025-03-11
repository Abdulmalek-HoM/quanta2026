package VisionColorDetection;

import org.opencv.core.*;

import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.List;

import org.openftc.easyopencv.OpenCvPipeline;

public class Pipeline_of_detecting_yellow_color extends OpenCvPipeline {

    public Scalar lowerRGB = new Scalar(13.0, 148.0, 227.0, 0.0);
    public Scalar upperRGB = new Scalar(35.0, 195.0, 246.0, 0.0);
    private Mat rgbMat = new Mat();

    private Mat rgbBinaryMat = new Mat();

    private ArrayList<MatOfPoint> contours = new ArrayList<>();
    private Mat hierarchy = new Mat();

    private MatOfPoint biggestContour = null;

    public Scalar lineColor = new Scalar(255.0, 0.0, 0.0, 0.0);
    public int lineThickness = 3;

    private Mat rgbBinaryMatContours = new Mat();

    private ArrayList<MatOfPoint> contours1 = new ArrayList<>();
    private Mat hierarchy1 = new Mat();

    public Scalar lineColor1 = new Scalar(0.0, 255.0, 0.0, 0.0);
    public int lineThickness1 = 3;

    private Mat inputContours = new Mat();

    @Override
    public Mat processFrame(Mat input) {


        Imgproc.cvtColor(input, rgbMat, Imgproc.COLOR_RGBA2RGB);
        Core.inRange(rgbMat, lowerRGB, upperRGB, rgbBinaryMat);

//        contours.clear();
//        hierarchy.release();
        Mat yellowMask = preprocessFrame(input);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        this.biggestContour = null;
        for(MatOfPoint contour : contours) {
            if((biggestContour == null) || (Imgproc.contourArea(contour) > Imgproc.contourArea(biggestContour))) {
                this.biggestContour = contour;
            }
        }

//        rgbBinaryMat.copyTo(rgbBinaryMatContours);

        ArrayList<MatOfPoint> contoursList = new ArrayList<>();
        if(biggestContour != null) {
            contoursList.add(biggestContour);
        }

        Imgproc.drawContours(rgbBinaryMatContours, contoursList, -1, lineColor, lineThickness);

//        contours1.clear();
//        hierarchy1.release();
//        Imgproc.findContours(rgbBinaryMat, contours1, hierarchy1, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
//
//        input.copyTo(inputContours);
//        Imgproc.drawContours(inputContours, contours1, -1, lineColor1, lineThickness1);
//
        return rgbBinaryMatContours;
//        return rgbBinaryMatContours;
//        return inputContours;
    }
    private Mat preprocessFrame(Mat frame) {
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

        Scalar lowerYellow = new Scalar(100, 100, 100);
        Scalar upperYellow = new Scalar(180, 255, 255);


        Mat yellowMask = new Mat();
        Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

        return yellowMask;
    }

}


