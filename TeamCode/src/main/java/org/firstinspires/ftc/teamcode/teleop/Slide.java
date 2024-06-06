package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Slide {

    double Kp = 0.0035;
    double Kd = 0.000000001;

    double lastError = 0;

    // Elapsed timer class from SDK, please use it, it's epic
    ElapsedTime timer = new ElapsedTime();

public void PIDcontrol (double reference, DcMotorEx leftSlide, DcMotorEx rightSlide){


        // obtain the encoder position
        int encoderPosition = leftSlide.getCurrentPosition();
        // calculate the error
        double error = reference - encoderPosition;

        // rate of change of the error
        double derivative = (error - lastError) / timer.seconds();

        double out = (Kp * error) + (Kd * derivative);

        leftSlide.setPower(out);
        rightSlide.setPower(out);

        lastError = error;

        // reset the timer for next time
        timer.reset();

    }

}
