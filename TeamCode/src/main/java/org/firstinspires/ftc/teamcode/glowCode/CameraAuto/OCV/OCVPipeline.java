package org.firstinspires.ftc.teamcode.glowCode.CameraAuto.OCV;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
//import org.firstinspires.ftc.teamcode.glowCode.CameraAuto.OpenCVStuff;

public class OCVPipeline extends OpenCvPipeline {
   static int lastResult;


    @Override
    public Mat processFrame(Mat input)
    {
        Mat returnMat = detectNumber(input);
        return returnMat;

    }



    public static Mat makeEdged(Mat inputMat) {
        Mat grayscaleImage = new Mat();
        Imgproc.cvtColor(inputMat, grayscaleImage, Imgproc.COLOR_BGR2GRAY);
        Mat blurredImage = new Mat();
        Imgproc.GaussianBlur(grayscaleImage, blurredImage, new Size(5, 5), 0);
        Mat edgedImage = new Mat();
        Imgproc.Canny(blurredImage, edgedImage, 50, 200, 255);
        return edgedImage;
    }

    public static MatOfPoint findLargestContour(Mat image) {
        List<MatOfPoint> contour = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(image, contour, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        double largestContourSize = 0;
        int largestContour = 0;
        for (int i = 0; i < contour.size(); i++) {
            if (Imgproc.contourArea(contour.get(i)) > largestContourSize) {
                largestContour = i;
                largestContourSize = Imgproc.contourArea(contour.get(i));
            }
        }
        return contour.get(largestContour);
    }

    public static Mat detectNumber(Mat inputtedImage) {
        Mat image1 = Imgcodecs.imread("![](../../../../../../../../../FtcRobotController/src/main/assets/IMG_2811.jpg)");
        Mat image2 = Imgcodecs.imread("![](../../../../../../../../../FtcRobotController/src/main/assets/IMG_2810.jpg)");
        Mat image3 = Imgcodecs.imread("![](../../../../../../../../../FtcRobotController/src/main/assets/IMG_2812.jpg)");
        Mat comparableImage1 = makeEdged(image1);
        Mat comparableImage2 = makeEdged(image2);
        Mat comparableImage3 = makeEdged(image3);
        /*MatOfPoint contour1 = findLargestContour(comparableImage1);
        MatOfPoint contour2 = findLargestContour(comparableImage2);
        MatOfPoint contour3 = findLargestContour(comparableImage3);*/
        Mat edgedImage = makeEdged(inputtedImage);
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edgedImage, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        final int magicNumber = 100;
        for (int i = 0; i < contours.size(); i++) {
            if (Imgproc.contourArea(contours.get(i)) > magicNumber) {
                if (Imgproc.matchShapes(edgedImage, comparableImage1, 1, 1) < 0.5) {
                    lastResult=1;
                    return edgedImage;
                } else if (Imgproc.matchShapes(edgedImage, comparableImage2, 1, 1) < 0.5) {
                   lastResult =2;
                    return edgedImage;
                } else if (Imgproc.matchShapes(edgedImage, comparableImage3, 1, 1) < 0.5) {
                    lastResult = 3;
                    return edgedImage;
                }
                return edgedImage;
            }
        }
        return edgedImage;
    }

    public int getLatestResults()
    {
        return lastResult;
    }
}

