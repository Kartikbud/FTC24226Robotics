package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotFunctions extends LinearOpMode {
    public DcMotor leftSlide, rightSlide;
    //public Servo leftArm, rightArm;
    public int slideDownPos = 0;
    public int slideUpPos = 1600;

    public int armUpPos = 1;
    public int armDownPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
    }

    public void intialize() {
        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setTargetPosition(slideDownPos);
        rightSlide.setTargetPosition(slideDownPos);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void slideUp() {
        leftSlide.setTargetPosition(slideUpPos);
        rightSlide.setTargetPosition(slideUpPos);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void slideDown() {
        leftSlide.setTargetPosition(slideDownPos);
        rightSlide.setTargetPosition(slideDownPos);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void armUp(Servo left, Servo right) {
        left.setPosition(0.4);
        right.setPosition(0.4);
    }


}
