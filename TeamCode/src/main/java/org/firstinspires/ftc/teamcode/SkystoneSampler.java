package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;

public class SkystoneSampler extends OpenCvPipeline {

    // These are the mats we need, I will be explaining them as we go
    private Mat matYCrCb = new Mat();
    private ArrayList<Mat> matCb = new ArrayList<>();
    private ArrayList<Mat> stones = new ArrayList<>();
    private ArrayList<Integer> order = new ArrayList();
    private ArrayList<Scalar> means = new ArrayList();
    ArrayList<Integer> skystones = new ArrayList<>();

    //These will store the Cb values
    public int leftX;
    public int rightX;
    int[] indexes = {0, 0};
    Rect[] blocks;
    public SkystoneSampler(Rect... blocks) {
        this.blocks = blocks;
    }
    //These will be the points for our rectangle


    /**
     * This will create the rectangles
     * @param frame the input mat
     * @param points the points for the rectangle
     * @param color the color of the rectangle when it is displayed on screen
     * @param thickness the thickness of the rectangle
     */
    public Mat drawRectangle(Mat frame,Rect rect,Scalar color,int thickness){
        Imgproc.rectangle(frame, rect, color, thickness);
        //submat simply put is cropping the mat
        return frame.submat(rect);

    }

    @Override
    public Mat processFrame(Mat input) {
        /**
         *input which is in RGB is the frame the camera gives
         *We convert the input frame to the color space matYCrCb
         *Then we store this converted color space in the mat matYCrCb
         *For all the color spaces go to
         *https://docs.opencv.org/3.4/d8/d01/group__imgproc__color__conversions.html
         */
        Imgproc.cvtColor(input, matYCrCb, Imgproc.COLOR_RGB2YCrCb);

        for(Rect stone: blocks) {
            order.add(stone.x);
            stones.add(drawRectangle(matYCrCb, stone, new Scalar (255, 0, 255), 2));
            matCb.add(new Mat());
        }

        /**
         *This will extract the value of the CB channel in both rectangles
         *0 is the Y channel, 1 is the Cr, 2 is Cb
         */

        for(int i = 0; i < stones.size(); i++) {
            Core.extractChannel(stones.get(i), matCb.get(i), 2);
            means.add(Core.mean(matCb.get(i)));
        }

        Scalar max = means.get(0);
        int biggestIndex = 0;

        for (Scalar k : means) {
            if (k.val[0] > max.val[0]) {
                max = k;
                biggestIndex = means.indexOf(k);
            }
        }

        int secondBiggestIndex = 0;
        Scalar secondMax = means.get(0);
        for (Scalar k : means) {
            if ((k.val[0] > secondMax.val[0]) && (secondMax.val[0] != max.val[0]))  {
                secondMax = k;
                secondBiggestIndex = means.indexOf(k);
            }
        }

        skystones.add(order.get(biggestIndex));
        skystones.add(order.get(secondBiggestIndex));
        indexes[0] = biggestIndex;
        indexes[1] = secondBiggestIndex;
        Collections.sort(skystones);

        leftX = skystones.get(0);
        rightX = skystones.get(1);

        order.clear();
        stones.clear();
        matCb.clear();
        means.clear();
        skystones.clear();

        for(Mat mat: stones) {
            mat.release();
        }
        return matYCrCb;
    }

    public int getLeftSkystoneX() {
        return leftX;
    }

    public int getRightSkystoneX() {
        return rightX;
    }

    public int[] getIndex() {
        return indexes;
    }


}
