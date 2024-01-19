package org.firstinspires.ftc.teamcode.auto;



import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotFunctions extends LinearOpMode {
    DcMotor rightFront, rightRear, leftFront, leftRear;
    public Servo testServo;

    @Override
    public void runOpMode() throws InterruptedException {
        testServo = hardwareMap.get(Servo.class, "servo");
    }

    public void servo() {
        testServo.setPosition(0.2);
    }
}
