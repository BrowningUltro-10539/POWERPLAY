package org.firstinspires.ftc.teamcode.vision.Pole;



import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


public class PoleDetector {
    private OpenCvWebcam webcam;
    private PoleDetectionPipeline opencv = null;
    Telemetry telemetry;

    public PoleDetector(LinearOpMode opMode, Telemetry telemetry) {
        //initialize webcam
        this.telemetry = telemetry;
        webcam = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"));
    }

    public void findPole() {

        opencv = new PoleDetectionPipeline(telemetry);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.setPipeline(opencv);
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        FtcDashboard.getInstance().startCameraStream(webcam, 60);
    }

    public double centerX() {
        return opencv.poleRect.x;
    }

    public double centerY() {
        return opencv.poleRect.y;
    }

    public double poleCenterX(){
        return opencv.poleRect.x + opencv.poleRect.width / 2;
    }

    public void stopCamera() {
        webcam.stopStreaming();
    }
}
