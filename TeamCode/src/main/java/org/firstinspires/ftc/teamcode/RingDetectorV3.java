package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class RingDetectorV3 {
    OpenCvCamera phoneCam;
    HardwareMap hwmp;
    Telemetry telemetry;
    Double ringCount = 0.0;
    double rows, rect1Cols, rect2Cols;

    double topColor;
    double bottomColor;
    double standColor;


    public RingDetectorV3(String Color, HardwareMap hw, Telemetry telemetry, double rows, double rect1Cols, double rect2Cols){
        this.hwmp = hw;
        telemetry = telemetry;
        this.rows = rows;
        this.rect1Cols = rect1Cols;
        this.rect2Cols = rect2Cols;
    }


    public void init(){
        int cameraMonitorViewId = hwmp.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwmp.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.setPipeline(new RingDetectingPipeline());
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
            }
        });
    }

    public double getRingPosition(){
        return ringCount; }


    class RingDetectingPipeline extends OpenCvPipeline {


        Mat HSV = new Mat();
        Mat upperCrop = new Mat();
        Mat lowerCrop = new Mat();
        Mat standardCrop = new Mat();

        Mat outputMat = new Mat();


        @Override
        public Mat processFrame(Mat input) {
            //convert mat.
            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

            //copy input mat onto output
            input.copyTo(outputMat);

            double YCbCrRows = Math.round(HSV.rows()*rows);
            double Rect1YCbCrCols = Math.round(HSV.cols() * rect1Cols);
            double Rect2YCbCrCols = Math.round(HSV.cols() * rect2Cols);
            int width = 100;
            int height1 = 20;
            int height2 = 10;
            int height = height1-height2;
            Rect rect = new Rect(25,310, width, height1);
            Rect rect2 = new Rect(25, 345, width, height2);
            Rect rect3 = new Rect(151, 400, 50, 50);

            System.out.println(String.format("Width: %d, Height: %d, Rows: %f, Cols1: %f, Cols2: %f",  width, height, YCbCrRows,Rect1YCbCrCols,Rect2YCbCrCols));

//draw rectangles so we can visualize them
            Imgproc.rectangle(outputMat, rect, new Scalar(0, 0, 255), 2);
            Imgproc.rectangle(outputMat, rect2, new Scalar(0, 0, 255), 2);
            Imgproc.rectangle(outputMat, rect3, new Scalar(0, 0, 255), 2);

            System.out.println(String.format("hello there"));


            //crop the top part of the stack, and the bottom part of the stack.
            standardCrop = HSV.submat(rect3);
            lowerCrop = HSV.submat(rect2);
            upperCrop = HSV.submat(rect);
            System.out.println(String.format("general kenobi"));


            //extract the Hue channel
            Core.extractChannel(lowerCrop, lowerCrop, 2);
            Core.extractChannel(upperCrop, upperCrop, 2);
            Core.extractChannel(standardCrop, standardCrop, 2);
            System.out.println(String.format("you are a bold one"));


            //take average
            Scalar lowerColor = Core.mean(lowerCrop);
            Scalar upperColor = Core.mean(upperCrop);
            Scalar standardColor = Core.mean(standardCrop);
            System.out.println(String.format("*lightsaber noises*"));


            bottomColor = lowerColor.val[0];
            topColor = upperColor.val[0];
            standColor = standardColor.val[0];
            //0-60 = orange

            //check which one to determine the

            if (topColor < standColor) {
                ringCount = 4.0;}
            else if (bottomColor < standColor && topColor >= standColor){
                ringCount = 1.0;}
            else if (bottomColor > standColor) {
                ringCount = 0.0;
            }

            Imgproc.putText(outputMat,
                    "Rings : " + ringCount.toString(),
                    new Point(rect.x + 100, rect.y + 200),
                    3,
                    .75,
                    new Scalar(0, 0, 255),
                    1);
            Imgproc.putText(outputMat,
                    "Upper Hue : " + lowerColor.toString(),
                    new Point(rect.x + 100, rect.y + 100),
                    3,
                    .75,
                    new Scalar(0, 0, 255),
                    1);
            Imgproc.putText(outputMat,
                    "Lower Hue : " + upperColor.toString(),
                    new Point(rect.x + 100, rect.y + 150),
                    3,
                    .75,
                    new Scalar(0, 0, 255),
                    1);
            Imgproc.putText(outputMat,
                    "Lower Hue : " + standardColor.toString(),
                    new Point(rect.x + 100, rect.y + 150),
                    3,
                    .75,
                    new Scalar(0, 0, 255),
                    1);

            return outputMat;
        }
    }
    public double getVal(){
        return topColor;
    }

    public double getVal2(){
        return bottomColor;
    }
}
