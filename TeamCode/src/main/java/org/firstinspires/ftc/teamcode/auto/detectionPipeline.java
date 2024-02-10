package org.firstinspires.ftc.teamcode.auto;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;
import java.util.List;

public class detectionPipeline extends OpenCvPipeline {

    List<Integer> red = Arrays.asList(255, 0, 0);
    List<Integer> blue = Arrays.asList(0, 0, 255);

    List<Integer> ELEMENT_COLOR = blue;

    String color_zone = "Center";

    Mat original;

    Mat rightZone;
    Mat centerZone;

    Scalar avgColorRight;
    Scalar avgColorCenter;

    double distanceRight = 1;
    double distanceCenter = 1;

    double maxDistance = 0;

    int toggleShow = 1;

    @Override
    public Mat processFrame(Mat input) {
        original = input.clone();

        centerZone = input.submat(new Rect(230, 400, 180, 190));
        rightZone = input.submat(new Rect(790, 505, 200, 240));

        avgColorCenter = Core.mean(centerZone);
        avgColorRight = Core.mean(rightZone);

        //Putting averaged colors on zones (we can see on camera now)
        centerZone.setTo(avgColorCenter);
        rightZone.setTo(avgColorRight);

        distanceCenter = color_distance(avgColorCenter, ELEMENT_COLOR);
        distanceRight = color_distance(avgColorRight, ELEMENT_COLOR);

        if ((distanceCenter > 195) && (distanceRight > 190)){
            color_zone = "Left";
            maxDistance = -1;
        }else{
            maxDistance = Math.min(distanceCenter, distanceRight);

            if (maxDistance == distanceCenter) {
                //telemetry.addData("Zone 1 Has Element", distanceCenter);
                color_zone = "Center";

            }else{
                //telemetry.addData("Zone 2 Has Element", distanceRight);
                color_zone = "Right";
            }
        }

        // Allowing for the showing of the averages on the stream
        if (toggleShow == 1){
            return input;
        }else{
            return original;
        }
    }

    public double color_distance(Scalar color1, List color2){
        double r1 = color1.val[0];
        double g1 = color1.val[1];
        double b1 = color1.val[2];

        int r2 = (int) color2.get(0);
        int g2 = (int) color2.get(1);
        int b2 = (int) color2.get(2);

        return Math.sqrt(Math.pow((r1 - r2), 2) + Math.pow((g1 - g2), 2) + Math.pow((b1 - b2), 2));
    }

    public void setAlliancePipe(String alliance){
        if (alliance.equals("red")){
            ELEMENT_COLOR = red;
        }else{
            ELEMENT_COLOR = blue;
        }
    }

    public String get_element_zone(){
        return color_zone;
    }

    public double getMaxDistance(){
        return maxDistance;
    }

    public void setToggleShow () {
        toggleShow = toggleShow*-1;
    }

}
