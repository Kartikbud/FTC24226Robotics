package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Subsystem {

    DcMotor leftSlide, rightSlide;
    Servo leftArm, rightArm;

    Servo leftClaw, rightClaw;
    public int slideDownPos = 0;
    public int slideUpPos = 1600;
    public double armDownPos = 0;
    public double armUpPos = 0.583;

    double clawClosed = 1;
    double clawOpen = 0.85;


    public Subsystem (HardwareMap hardwareMap) {
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setTargetPosition(slideDownPos);
        rightSlide.setTargetPosition(slideDownPos);
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
    }

    public void armUp() {
        leftArm.setPosition(armUpPos);
        rightArm.setPosition(armUpPos);
    }

    public void armDown() {
        leftArm.setPosition(armDownPos);
        rightArm.setPosition(armDownPos);
    }

    public void armPositionTo(double position) {
        position = constrain((double) position, (double) armDownPos, (double) armUpPos);
        leftArm.setPosition(position);
        rightArm.setPosition(position);
    }

    public void slideUp() {
        leftSlide.setTargetPosition(slideUpPos);
        rightSlide.setTargetPosition(slideUpPos);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(0.3);
        rightSlide.setPower(0.3);
    }

    public void slideDown() {
        leftSlide.setTargetPosition(slideDownPos);
        rightSlide.setTargetPosition(slideDownPos);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(0.3);
        rightSlide.setPower(0.3);
    }

    public void slidePositionTo(int position) {
        position = constrain((int) position, (int) slideDownPos, (int) slideUpPos);
        leftSlide.setTargetPosition(position);
        rightSlide.setTargetPosition(position);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(0.3);
        rightSlide.setPower(0.3);

    }

    public void rightClawOpen () {
        rightClaw.setPosition(clawOpen);
    }

    public void rightClawClosed () {
        rightClaw.setPosition(clawClosed);
    }

    public void leftClawOpen () {
        leftClaw.setPosition(clawOpen);
    }

    public void leftClawClosed () {
        leftClaw.setPosition(clawClosed);
    }

    public double constrain(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }
    public int constrain(int value, int min, int max) {
        return Math.min(Math.max(value, min), max);
    }
}
