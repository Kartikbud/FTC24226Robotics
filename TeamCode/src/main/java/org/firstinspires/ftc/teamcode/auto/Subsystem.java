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

    DcMotor leftSlide;
    DcMotor rightSlide;

    Servo leftArm;
    Servo rightArm;

    public int slideDownPos = 0;
    public int slideUpPos = 1600;

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
    }

    public void armUp() {
        leftArm.setPosition(0.4);
        rightArm.setPosition(0.4);
    }

    public void armDown() {
        leftArm.setPosition(0);
        rightArm.setPosition(0);
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
}
