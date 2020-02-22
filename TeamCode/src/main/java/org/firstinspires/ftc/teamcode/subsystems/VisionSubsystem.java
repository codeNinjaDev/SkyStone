package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.vision.SkystoneDetector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.libs.SkystoneDetectorEx;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class VisionSubsystem implements Subsystem {
    SkystoneDetectorEx detector;
    OpenCvCamera webcam;


    public VisionSubsystem(HardwareMap hw) {
        int cameraMonitorViewId = hw.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hw.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        /*
         * Open the connection to the camera device
         */
        webcam.openCameraDevice();

        detector = new SkystoneDetectorEx(63,25, 30, 50, 50);
        webcam.setPipeline(detector);

        webcam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_RIGHT);
    }

    public SkystoneDetector.SkystonePosition getSkystonePosition() {
        return detector.getSkystonePosition();
    }

    @Override
    public void initialize() {

    }

    @Override
    public void reset() {

    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        webcam.stopStreaming();
    }

    @Override
    public void disable() {
        webcam.closeCameraDevice();
    }
}
