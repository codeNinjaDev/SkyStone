package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.ArrayList;
import java.util.List;

class SkystoneDetector {


    OpenCvCamera phoneCamera;
    int cameraMonitorViewId;
    SkystonePipeline pipeline;

    public SkystoneDetector(HardwareMap hw, int width, int height, OpenCvCameraRotation rotation) {
        cameraMonitorViewId = hw.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hw.appContext.getPackageName());
        phoneCamera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCamera.openCameraDevice();

        pipeline = new SkystonePipeline();

        phoneCamera.setPipeline(pipeline);
        phoneCamera.startStreaming(width, height, rotation);
    }

    public double getLeftSkystoneX() {
        return pipeline.leftX;
    }

    public double getRightStoneX() {
        return pipeline.rightX;
    }

    public Rect getLeftSkystone() {
        return pipeline.leftSkystone;
    }

    public Rect getRightSkystone() {
        return pipeline.rightSkystone;
    }

    public Rect getYellowBox() {
        return pipeline.yellowBox;
    }

    static class SkystonePipeline extends OpenCvPipeline {
        public double leftX, rightX = 0;
        public Rect leftSkystone, rightSkystone, yellowBox = null;
        private Mat invert, hsvYellow, hsvBlack;
        private ArrayList<MatOfPoint> yellowContours = new ArrayList<MatOfPoint>();
        private ArrayList<MatOfPoint> blackContours = new ArrayList<MatOfPoint>();
        private MatOfPoint yellowBlocks;
        @Override
        public Mat processFrame(Mat input) {

            yellowContours.clear();
            blackContours.clear();

            invert = new Mat();
            Core.bitwise_not(input, invert);
            int kernelSize = 2 * 7 + 1;
            Imgproc.blur(invert.clone(), invert, new Size(kernelSize, kernelSize));
            hsvYellow = invert.clone();
            double[] hsvYellowHue = {100, 130};
            double[] hsvYellowSaturation = {120, 230};
            double[] hsvYellowValue = {120, 255};

            hsvThreshold(hsvYellow, hsvYellowHue, hsvYellowSaturation, hsvYellowValue);
            Mat dilateKernel1 = new Mat();
            Point dilateAnchor1 = new Point(-1, 1);
            Imgproc.dilate(hsvYellow, hsvYellow, dilateKernel1, dilateAnchor1, 9, Core.BORDER_CONSTANT);
            findContours(hsvYellow.clone(), false, yellowContours);
            try {
                yellowBlocks = findMaxCnt(yellowContours);

                yellowBox = Imgproc.boundingRect(yellowBlocks);
                Rect newSize = new Rect(yellowBox.x, 0, yellowBox.x + yellowBox.width, invert.height());

                Mat ROI = new Mat(invert, newSize);
                Imgproc.rectangle(hsvYellow, yellowBox, new Scalar(255, 0, 0), 3);


                hsvBlack = ROI.clone();
                double[] hsvBlackHue = {84, 105};
                double[] hsvBlackSaturation = {0, 42};
                double[] hsvBlackValue = {188, 255};

                hsvThreshold(hsvBlack, hsvBlackHue, hsvBlackSaturation, hsvBlackValue);
                Mat dilateKernel = new Mat();
                Point dilateAnchor = new Point(-1, 1);
                Imgproc.dilate(hsvBlack.clone(), hsvBlack, dilateKernel, dilateAnchor, 19, Core.BORDER_CONSTANT);

                findContours(hsvBlack, false, blackContours);
                if(Imgproc.contourArea(findMaxCnt(blackContours)) < 80) {
                    return ROI;
                }
                MatOfPoint biggestSkystone = findMaxCnt(blackContours);
                Rect skystone1 = Imgproc.boundingRect(biggestSkystone);
                blackContours.remove(biggestSkystone);
                MatOfPoint secondBiggestSkystone = findMaxCnt(blackContours);
                Rect skystone2 = Imgproc.boundingRect(secondBiggestSkystone);
                Imgproc.rectangle(ROI, skystone1, new Scalar(255, 0, 0), 3);
                Imgproc.rectangle(ROI, skystone2, new Scalar(255, 0, 0), 3);

                if (skystone1.x < skystone2.x) {
                    leftX = centerX(skystone1);
                    rightX = centerX(skystone2);
                    leftSkystone = skystone1;
                    rightSkystone = skystone2;
                } else {
                    leftX = centerX(skystone2);
                    rightX = centerX(skystone1);
                    leftSkystone = skystone2;
                    rightSkystone = skystone1;
                }
                return hsvYellow;
            } catch (Exception e) {
                return hsvYellow;
            }
        }

        private void hsvThreshold(Mat frame, double[] hue, double[] sat, double[] val) {
            Imgproc.cvtColor(frame.clone(), frame, Imgproc.COLOR_BGR2HSV);
            Core.inRange(frame.clone(), new Scalar(hue[0], sat[0], val[0]), new Scalar(hue[1], sat[1], val[1]), frame);
        }
        private void findContours(Mat input, boolean externalOnly,
                                  List<MatOfPoint> contours) {
            Mat hierarchy = new Mat();
            contours.clear();
            int mode;
            if (externalOnly) {
                mode = Imgproc.RETR_EXTERNAL;
            }
            else {
                mode = Imgproc.RETR_LIST;
            }
            int method = Imgproc.CHAIN_APPROX_SIMPLE;
            Imgproc.findContours(input, contours, hierarchy, mode, method);
        }

        private MatOfPoint findMaxCnt(List<MatOfPoint> contours) {
            MatOfPoint largestCnt = null;
            double maxArea = 0;
            for(MatOfPoint cnt: contours) {
                double currArea = Imgproc.contourArea(cnt);

                if(currArea > maxArea) {
                    largestCnt = cnt;
                    maxArea = currArea;
                }
            }
            return largestCnt;
        }

        private double centerX(Rect rect) {
            return (2*rect.x + rect.width) / 2;
        }
    }
}


