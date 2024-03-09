package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends LinearOpMode {

    DcMotor rightFront, rightRear, leftFront, leftRear;
    DcMotor leftSlide, rightSlide;
    Servo leftArm, rightArm;
    Servo leftClaw, rightClaw;
    Servo leftClawRotate, rightClawRotate;
    Servo drone;
    IMU imu;
    Orientation myRobotOrientation;
    double initYaw;
    double axial_drive;
    double lateral_drive;
    double yaw_drive;


    //constants
    double DRIVE_POWER_SCALE = 0.8;
    double SLIDE_POWER_SCALE = 0.25;
    double FINE_DRIVE_POWER_SCALE = DRIVE_POWER_SCALE/3;
    double armUpPos = 0.65;
    double armDownPos = 0.02;
    double clawClosedPos = 1;
    double clawOpenPos = 0.85;
    double droneLockPos = 0.23;
    double droneUnlockPos = 1;
    int slideMaxPos = 600;
    int slideMinPos = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        //drive initialization
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class,"rightRear");
        leftFront = hardwareMap.get(DcMotor.class,"leftFront");
        leftRear = hardwareMap.get(DcMotor.class,"leftRear");
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //slide initialization
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setDirection(DcMotor.Direction.FORWARD);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //arm initialization
        leftArm = hardwareMap.get(Servo.class, "leftArm");
        rightArm = hardwareMap.get(Servo.class, "rightArm");
        leftArm.setDirection(Servo.Direction.FORWARD);
        rightArm.setDirection(Servo.Direction.REVERSE);

        //claw initialization
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        rightClaw.setDirection(Servo.Direction.FORWARD);
        leftClaw.setDirection(Servo.Direction.REVERSE);

        //claw rotate initialization
        leftClawRotate = hardwareMap.get(Servo.class, "leftClawRotate");
        rightClawRotate = hardwareMap.get(Servo.class, "rightClawRotate");
        rightClawRotate.setDirection(Servo.Direction.FORWARD);
        leftClawRotate.setDirection(Servo.Direction.REVERSE);


        //drone initialization
        drone = hardwareMap.get(Servo.class, "drone");
        drone.setDirection(Servo.Direction.REVERSE);

        //imu initialization
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        initYaw = myRobotOrientation.firstAngle;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            //drive
            lateral_drive = gamepad1.left_stick_x;
            yaw_drive = gamepad1.right_stick_x;
            if (gamepad1.right_trigger > 0 ) {
                axial_drive = gamepad1.right_trigger;
            } else if (gamepad1.left_trigger > 0) {
                axial_drive = -gamepad1.left_trigger;
            } else {
                axial_drive = -gamepad1.left_stick_y;
            }

            //fine drive
            if (gamepad1.dpad_down) {
                axial_drive = -FINE_DRIVE_POWER_SCALE;
            }
            if (gamepad1.dpad_left) {
                lateral_drive = -FINE_DRIVE_POWER_SCALE;
            }
            if (gamepad1.dpad_right) {
                lateral_drive = FINE_DRIVE_POWER_SCALE;
            }
            if (gamepad1.dpad_up) {
                axial_drive = FINE_DRIVE_POWER_SCALE;
            }
            if (gamepad1.right_bumper) {
                yaw_drive = FINE_DRIVE_POWER_SCALE;
            }
            if (gamepad1.left_bumper) {
                yaw_drive = -FINE_DRIVE_POWER_SCALE;
            }

            //slide
            if (gamepad2.b){
                slide(slideMaxPos);
            }
            if (gamepad2.a){
                slide(slideMaxPos/2);
            }
            if (gamepad2.left_stick_button){
                slide(slideMinPos);
            }

            //arm  NEED TO ADD MORE
            if (gamepad2.x) {
                armDown();
            } else {
                armUp();
            }

            //drone
            if (gamepad1.y) {
                droneUnlock();
            } else {
                droneLock();
            }

            //claw
            if (gamepad2.right_bumper) {
                rightClawOpen();
            } else {
                rightClawClosed();
            }
            if (gamepad2.left_bumper) {
                leftClawOpen();
            } else {
                leftClawClosed();
            }


            if (gamepad2.dpad_down) {
                clawRotate(0.5);
            }
            if (gamepad2.dpad_up){
                clawRotate(0);
            }


            mecanum_drive_robot(axial_drive,lateral_drive,yaw_drive);


            telemetry.addData("right", rightArm.getPosition());
            telemetry.addData("left", leftArm.getPosition());
            telemetry.update();

        }
    }
    public void mecanum_drive_field(double axial, double lateral, double yaw, double heading) {

        double zeroedYaw = -initYaw + heading;
        double theta = Math.atan2(axial, lateral) * 180/Math.PI;
        double realTheta = (360 - zeroedYaw) + theta;
        double power = Math.hypot(lateral, axial);

        double sin = Math.sin((realTheta * (Math.PI / 180)) - (Math.PI / 4));
        double cos = Math.cos((realTheta * (Math.PI / 180)) - (Math.PI / 4));
        double maxSinCos = Math.max(Math.abs(sin), Math.abs(cos));

        double leftFrontPower = (power * cos / maxSinCos + yaw);
        double leftRearPower = (power * sin / maxSinCos + yaw);
        double rightFrontPower = (power * sin / maxSinCos - yaw);
        double rightRearPower = (power * cos / maxSinCos - yaw);

        if ((power + Math.abs(yaw)) > 1) {
            leftFrontPower /= power + yaw;
            leftRearPower /= power + yaw;
            rightFrontPower /= power - yaw;
            rightRearPower /= power - yaw;
        }

        leftFront.setPower(leftFrontPower * DRIVE_POWER_SCALE);
        leftRear.setPower(leftRearPower * DRIVE_POWER_SCALE);
        rightFront.setPower(rightFrontPower * DRIVE_POWER_SCALE);
        rightRear.setPower(rightRearPower * DRIVE_POWER_SCALE);
    }

    public void mecanum_drive_robot(double axial, double lateral, double yaw) {
        double denominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);
        double leftFrontPower = (axial + lateral + yaw) / denominator;
        double leftRearPower = (axial - lateral + yaw) / denominator;
        double rightFrontPower = (axial - lateral - yaw) / denominator;
        double rightRearPower = (axial + lateral - yaw) / denominator;
        leftFront.setPower(leftFrontPower * DRIVE_POWER_SCALE);
        leftRear.setPower(leftRearPower * DRIVE_POWER_SCALE);
        rightFront.setPower(rightFrontPower * DRIVE_POWER_SCALE);
        rightRear.setPower(rightRearPower * DRIVE_POWER_SCALE);
    }

    public void slide(int position) {
        leftSlide.setTargetPosition(position);
        rightSlide.setTargetPosition(position);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(SLIDE_POWER_SCALE);
        rightSlide.setPower(SLIDE_POWER_SCALE);
    }

    public void armUp() {
        leftArm.setPosition(armUpPos);
        rightArm.setPosition(armUpPos);
    }

    public void armDown() {
        leftArm.setPosition(armDownPos);
        rightArm.setPosition(armDownPos);
    }

    public void armVarPos(double position) {
        leftArm.setPosition(position);
        rightArm.setPosition(position);
    }

    public void droneLock() {
        drone.setPosition(droneLockPos);
    }

    public void droneUnlock() {
        drone.setPosition(droneUnlockPos);
    }

    public void rightClawOpen() {
        rightClaw.setPosition(clawOpenPos);
    }

    public void rightClawClosed() {
        rightClaw.setPosition(clawClosedPos);
    }

    public void leftClawOpen() {
        leftClaw.setPosition(clawOpenPos);
    }

    public void leftClawClosed() {
        leftClaw.setPosition(clawClosedPos);
    }

    public void rightClaw (double position) {
        rightClaw.setPosition(position);
    }

    public void leftClaw (double position) {
        leftClaw.setPosition(position);
    }

    public void clawRotate (double position) {
        leftClawRotate.setPosition(position);
        rightClawRotate.setPosition(position);
    }

}
