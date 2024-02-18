package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="ThresholdTest")

public class ThresholdTest extends LinearOpMode{

    public String element_zone = "Center";

    private CameraSubsytem cameraDetection = null;

    boolean togglePreview = true;

    public void HardwareStart() {
        telemetry.addData("Object Creation", "Start");
        telemetry.update();

        cameraDetection = new CameraSubsytem(hardwareMap);

        telemetry.addData("Object Creation", "Done");
        telemetry.update();
    }



    public void runOpMode(){

        HardwareStart();

        String curAlliance = "blue";

        while (!opModeIsActive() && !isStopRequested()){
            element_zone = cameraDetection.elementDetection(telemetry);
            telemetry.addData("getMaxDistance", cameraDetection.getMaxDistance());

            if (togglePreview && gamepad2.a){
                togglePreview = false;
                cameraDetection.toggleAverageZone();
            }else if (!gamepad2.a){
                togglePreview = true;
            }


            if (gamepad1.x){
                curAlliance = "blue";
            }else if (gamepad1.b){
                curAlliance = "red";
            }
            cameraDetection.setAlliance(curAlliance);
            telemetry.addData("Select Alliance (Gamepad1 X = Blue, Gamepad1 B = Red)", "");
            telemetry.addData("Current Alliance Selected : ", curAlliance.toUpperCase());


            telemetry.update();
        }

        telemetry.addData("Object", "Passed waitForStart");

        telemetry.update();

    }

}
