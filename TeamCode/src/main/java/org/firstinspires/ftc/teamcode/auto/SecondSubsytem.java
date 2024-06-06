package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SecondSubsytem {
    DcMotor leftSlide, rightSlide;
    Servo leftArm, rightArm;
    Servo leftClaw, rightClaw;
    Servo leftClawRotate, rightClawRotate;
    Servo drone;

    double SLIDE_POWER_SCALE = 0.5;

    double clawClosedPos = 1;
    double clawOpenPos = 0.8;

    double droneLockPos = 0.3;

    //from teleop
    double armUpPos = 0.4;
    double armDownPos = 0.045;
    double armPlacePos = 0.9;
    double armOffset = 0.005;

    double clawRotateDownPos = 0.63;
    double clawRotateUpPos = 0;
    double clawRotatePlacePos = 0.25;
    double clawRotatePlaceDownPos = 1;

    int slideMaxPos = 2850;
    int slideHookPos = 2450;
    int slideHighPos = 2350;
    int slideMidPos = 1850;
    int slideLowPos = 1250;
    int slideHangPos = 400;
    int slideMinPos = 0;


    public SecondSubsytem(HardwareMap hardwareMap) {
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setTargetPosition(0);
        rightSlide.setTargetPosition(0);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        rightArm = hardwareMap.get(Servo.class, "rightArm");
        leftArm = hardwareMap.get(Servo.class, "leftArm");
        leftArm.setDirection(Servo.Direction.FORWARD);
        rightArm.setDirection(Servo.Direction.REVERSE);

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        rightClaw.setDirection(Servo.Direction.FORWARD);
        leftClaw.setDirection(Servo.Direction.REVERSE);

        leftClawRotate = hardwareMap.get(Servo.class, "leftClawRotate");
        rightClawRotate = hardwareMap.get(Servo.class, "rightClawRotate");
        rightClawRotate.setDirection(Servo.Direction.FORWARD);
        leftClawRotate.setDirection(Servo.Direction.REVERSE);


        //drone initialization
        drone = hardwareMap.get(Servo.class, "drone");
        drone.setDirection(Servo.Direction.FORWARD);


    }

    public void armPos(double pos) {
        pos = constrain((double) pos, (double) 0, (double) 1);
        leftArm.setPosition(pos + armOffset);
        rightArm.setPosition(pos);
    }

    public void slidePos(int pos) {
        leftSlide.setTargetPosition(pos);
        rightSlide.setTargetPosition(pos);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(SLIDE_POWER_SCALE);
        rightSlide.setPower(SLIDE_POWER_SCALE);

    }

    public void lockDrone () {
        drone.setPosition(droneLockPos);
    }

    public void rightClawOpen () {
        rightClaw.setPosition(clawOpenPos);
    }

    public void rightClawClosed () {
        rightClaw.setPosition(clawClosedPos);
    }

    public void leftClawOpen () {
        leftClaw.setPosition(clawOpenPos);
    }

    public void leftClawClosed () {
        leftClaw.setPosition(clawClosedPos);
    }

    public void clawRotatePos (double pos) {
        leftClawRotate.setPosition(pos);
        rightClawRotate.setPosition(pos);
    }

    public double constrain(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }
    public int constrain(int value, int min, int max) {
        return Math.min(Math.max(value, min), max);
    }
}
