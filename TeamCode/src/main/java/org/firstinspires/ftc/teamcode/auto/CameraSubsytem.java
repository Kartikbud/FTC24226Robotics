package org.firstinspires.ftc.teamcode.auto;

import android.hardware.HardwareBuffer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


public class CameraSubsytem {

    OpenCvCamera camera;
    detectionPipeline detectionPipeline;

    int camW = 1280;
    int camH = 720;

    String zone = "Center";

    public CameraSubsytem (HardwareMap hardwareMap) {

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        detectionPipeline = new detectionPipeline();

        camera.setPipeline(detectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(camW, camH, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

    }

    public void setAlliance(String alliance){
        detectionPipeline.setAlliancePipe(alliance);
    }

    public String elementDetection(Telemetry telemetry) {
        zone = detectionPipeline.get_element_zone();
        telemetry.addData("Element Zone", zone);
        return zone;
    }

    public void toggleAverageZone(){
        detectionPipeline.setToggleShow();
    }

    public double getMaxDistance(){
        return detectionPipeline.getMaxDistance();
    }
}
