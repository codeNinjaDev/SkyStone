package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="CVOpMode", group = "Test")  // @Autonomous(...) is the other common choice
public class cvOpMode extends LinearOpMode {
    SkystoneFinder detector;
    public void runOpMode() {
        detector = new SkystoneFinder(hardwareMap, 640, 480, OpenCvCameraRotation.UPRIGHT);

        while(!isStarted() && !isStopRequested()) {

            try {


            } catch(Exception e) {

            }
            telemetry.update();
        }
    }

    class SkystoneFinder {


        OpenCvCamera phoneCamera;
        int cameraMonitorViewId;
        SkystoneInPipeline pipeline;

        public SkystoneFinder(HardwareMap hw, int width, int height, OpenCvCameraRotation rotation) {
            cameraMonitorViewId = hw.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hw.appContext.getPackageName());
            phoneCamera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
            phoneCamera.openCameraDevice();

            pipeline = new SkystoneInPipeline();

            phoneCamera.setPipeline(pipeline);
            phoneCamera.startStreaming(width, height, rotation);
        }

        /*public double getLeftSkystoneX() {
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
        }*/

        public class SkystoneInPipeline extends OpenCvPipeline {
            //Outputs
            private Mat cvBitwiseNotOutput = new Mat();
            private Mat blurOutput = new Mat();
            private Mat hsvThresholdOutput = new Mat();
            private Mat cvDilateOutput = new Mat();
            private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
            private ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<MatOfPoint>();

            @Override
            public Mat processFrame(Mat source0) {
                Mat cvBitwiseNotSrc1 = source0;
                cvBitwiseNot(cvBitwiseNotSrc1, cvBitwiseNotOutput);

                // Step Blur0:
                Mat blurInput = cvBitwiseNotOutput;
                double blurRadius = 6.6037740347520355;
                blur(blurInput, blurRadius, blurOutput);

                // Step HSV_Threshold0:
                Mat hsvThresholdInput = blurOutput;
                double[] hsvThresholdHue = {104.136686273616, 126.09880759471442};
                double[] hsvThresholdSaturation = {120.77338421087471, 220.1877133105802};
                double[] hsvThresholdValue = {120.7733820239417, 255.0};
                hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);
                if(true)
                    return hsvThresholdOutput;

                // Step CV_dilate0:
                Mat cvDilateSrc = hsvThresholdOutput;
                Mat cvDilateKernel = new Mat();
                Point cvDilateAnchor = new Point(-1, -1);
                double cvDilateIterations = 9.0;
                int cvDilateBordertype = Core.BORDER_CONSTANT;
                Scalar cvDilateBordervalue = new Scalar(-1);
                cvDilate(cvDilateSrc, cvDilateKernel, cvDilateAnchor, cvDilateIterations, cvDilateBordertype, cvDilateBordervalue, cvDilateOutput);

                // Step Find_Contours0:
                Mat findContoursInput = cvDilateOutput;
                boolean findContoursExternalOnly = false;
                findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);

                return null;
            }

            /**
             * This method is a generated getter for the output of a CV_bitwise_not.
             * @return Mat output from CV_bitwise_not.
             */
            public Mat cvBitwiseNotOutput() {
                return cvBitwiseNotOutput;
            }

            /**
             * This method is a generated getter for the output of a Blur.
             * @return Mat output from Blur.
             */
            public Mat blurOutput() {
                return blurOutput;
            }

            /**
             * This method is a generated getter for the output of a HSV_Threshold.
             * @return Mat output from HSV_Threshold.
             */
            public Mat hsvThresholdOutput() {
                return hsvThresholdOutput;
            }

            /**
             * This method is a generated getter for the output of a CV_dilate.
             * @return Mat output from CV_dilate.
             */
            public Mat cvDilateOutput() {
                return cvDilateOutput;
            }

            /**
             * This method is a generated getter for the output of a Find_Contours.
             * @return ArrayList<MatOfPoint> output from Find_Contours.
             */
            public ArrayList<MatOfPoint> findContoursOutput() {
                return findContoursOutput;
            }

            /**
             * This method is a generated getter for the output of a Filter_Contours.
             * @return ArrayList<MatOfPoint> output from Filter_Contours.
             */
            public ArrayList<MatOfPoint> filterContoursOutput() {
                return filterContoursOutput;
            }


            /**
             * Computes the per element inverse of an image.
             * @param src the image to invert.
             * @param dst the inversion of the input image.
             */
            private void cvBitwiseNot(Mat src, Mat dst) {
                Core.bitwise_not(src, dst);
            }

            /**
             * Softens an image using one of several filters.
             * @param input The image on which to perform the blur.
             * @param type The blurType to perform.
             * @param doubleRadius The radius for the blur.
             * @param output The image in which to store the output.
             */
            private void blur(Mat input, double doubleRadius,
                              Mat output) {
                int radius = (int)(doubleRadius + 0.5);
                int kernelSize;
                kernelSize = 2 * radius + 1;
                Imgproc.blur(input, output, new Size(kernelSize, kernelSize));

            }

            /**
             * Segment an image based on hue, saturation, and value ranges.
             *
             * @param input The image on which to perform the HSL threshold.
             * @param hue The min and max hue
             * @param sat The min and max saturation
             * @param val The min and max value
             * @param output The image in which to store the output.
             */
            private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val,
                                      Mat out) {
                Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HSV);
                Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
                        new Scalar(hue[1], sat[1], val[1]), out);
            }

            /**
             * Expands area of higher value in an image.
             * @param src the Image to dilate.
             * @param kernel the kernel for dilation.
             * @param anchor the center of the kernel.
             * @param iterations the number of times to perform the dilation.
             * @param borderType pixel extrapolation method.
             * @param borderValue value to be used for a constant border.
             * @param dst Output Image.
             */
            private void cvDilate(Mat src, Mat kernel, Point anchor, double iterations,
                                  int borderType, Scalar borderValue, Mat dst) {
                if (kernel == null) {
                    kernel = new Mat();
                }
                if (anchor == null) {
                    anchor = new Point(-1,-1);
                }
                if (borderValue == null){
                    borderValue = new Scalar(-1);
                }
                Imgproc.dilate(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
            }

            /**
             * Sets the values of pixels in a binary image to their distance to the nearest black pixel.
             * @param input The image on which to perform the Distance Transform.
             * @param type The Transform.
             * @param maskSize the size of the mask.
             * @param output The image in which to store the output.
             */
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



}
