package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="CVOpMode", group = "Test")  // @Autonomous(...) is the other common choice
public class PixelFinder extends LinearOpMode {

    public Rect centerSquare(int x, int y, int radius) {
        return new Rect(x - (radius / 2), y - (radius / 2), radius, radius);
    }

    SkystoneFinder detector;
    public void runOpMode() {
        detector = new SkystoneFinder(hardwareMap, 640, 480, OpenCvCameraRotation.UPRIGHT);

        GamepadEx driver = new GamepadEx(gamepad1);

        ButtonReader moveUp, moveDown, moveLeft, moveRight;
        moveUp = new ButtonReader(driver, GamepadKeys.Button.DPAD_UP);
        moveDown = new ButtonReader(driver, GamepadKeys.Button.DPAD_DOWN);
        moveLeft = new ButtonReader(driver, GamepadKeys.Button.DPAD_LEFT);
        moveRight = new ButtonReader(driver, GamepadKeys.Button.DPAD_RIGHT);

        ButtonReader selectTarget = new ButtonReader(driver, GamepadKeys.Button.X);
        ButtonReader save = new ButtonReader(driver, GamepadKeys.Button.A);
        ArrayList<int[]> points = new ArrayList<int[]>();

        int[] currentPosition = {50, 50};
        while(!isStarted() && !isStopRequested()) {

            if(moveUp.wasJustPressed())
                currentPosition[1] += 10;
            else if(moveDown.wasJustPressed())
                currentPosition[1] -= 10;
            else if(moveRight.wasJustPressed())
                currentPosition[0] += 10;
            else if(moveDown.wasJustPressed())
                currentPosition[0] -= 10;
            else if (selectTarget.wasJustReleased()) {
                points.add(currentPosition);
                detector.pipeline.rects = new ArrayList<Rect>();

                for(int[] point: points) {
                    detector.pipeline.rects.add(centerSquare(point[0], point[1], 10));
                }
                currentPosition[0]  = 50;
                currentPosition[1] = 50;

            }
            telemetry.addData("Current Position", currentPosition);
            int i = 0;
            for(int[] point: points) {
                i++;
                telemetry.addData("Point " + i, point);
            }

            detector.pipeline.currPos = centerSquare(currentPosition[0], currentPosition[1], 5);


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
            public ArrayList<Rect> rects = new ArrayList<Rect>();
            public Rect currPos = centerSquare(50,50, 10);


            @Override
            public Mat processFrame(Mat source0) {
                Imgproc.rectangle(source0, currPos, new Scalar(255, 255, 255));

                for(Rect rect: rects) {
                    Imgproc.rectangle(source0, rect, new Scalar(0, 0, 255));
                }
                return source0;
            }

            /**
             * This method is a generated getter for the output of a CV_bitwise_not.
             * @return Mat output from CV_bitwise_not.
             */
            public Mat cvBitwiseNotOutput() {
                return cvBitwiseNotOutput;
            }


        }
    }



}
