package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends LinearOpMode {

    Slide lift = new Slide();

    DcMotor rightFront, rightRear, leftFront, leftRear;
    DcMotorEx leftSlide, rightSlide;
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
    double DRIVE_POWER_SCALE = 1;
    double SLIDE_POWER_SCALE = 0.75;
    double FINE_DRIVE_POWER_SCALE = DRIVE_POWER_SCALE/4;

    //arm
    double armUpPos = 0.42;
    double armDownPos = 0.045;
    double armPlacePos = 0.9;
    double armOffset = 0.005;
    double rightArmIncrement = -0.007;
    boolean outTake = false;

    //claw
    double clawClosedPos = 0.95;
    double clawOpenPos = 0.7;

    //claw rotate
    double clawRotateDownPos = 0.63;
    double clawRotateUpPos = 0;
    double clawRotatePlacePos = 0.25;
    double clawRotatePlaceDownPos = 1;

    //drone
    double droneLockPos = 0.3;
    double droneUnlockPos = 0.6;

    //slide
    int slidePos = 0;
    int slideMaxPos = 2850;
    int slideHookPos = 2450;
    int slideHighPos = 2450;
    int slideMidPos = 1850;
    int slideLowPos = 1000; //1450
    int slideHangPos = 700;
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
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setDirection(DcMotor.Direction.FORWARD);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        drone.setDirection(Servo.Direction.FORWARD);

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


            //DRIVER CONTROLS

            //drive
            lateral_drive = gamepad1.left_stick_x;
            yaw_drive = gamepad1.right_stick_x * 0.35;

            if (gamepad1.b) {
                yaw_drive = 0.8;
            }
            if (gamepad1.a) {
                yaw_drive = -0.8;
            }

            if (gamepad1.right_trigger > 0 && rightSlide.getCurrentPosition() <= slideMidPos && leftSlide.getCurrentPosition() <= slideMidPos) {
                axial_drive = gamepad1.right_trigger;
            } else if (gamepad1.left_trigger > 0 && rightSlide.getCurrentPosition() <= slideMidPos && leftSlide.getCurrentPosition() <= slideMidPos) {
                axial_drive = -gamepad1.left_trigger;
            } else if (rightSlide.getCurrentPosition() <= slideMidPos && leftSlide.getCurrentPosition() <= slideMidPos){
                axial_drive = -gamepad1.left_stick_y;
            } else if (rightSlide.getCurrentPosition() > slideMidPos && leftSlide.getCurrentPosition() > slideMidPos) {
                axial_drive = -gamepad1.left_stick_y * FINE_DRIVE_POWER_SCALE;
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
                lateral_drive = 1;
            }
            if (gamepad1.left_bumper) {
                lateral_drive = -1;
            }

            //drone
            if (gamepad1.y) {
                droneUnlock();
            } else {
                droneLock();
            }

            mecanum_drive_robot(axial_drive,lateral_drive,yaw_drive);



            //OPERATOR CONTROLS

            //slide
            if (gamepad2.dpad_right){
                slidePos = slideHighPos;

            }
            if (gamepad2.dpad_up){
                slidePos = slideMidPos;
            }
            if (gamepad2.dpad_left) {
                slidePos = slideLowPos;
            }
            if (gamepad2.left_stick_button || gamepad1.left_stick_button){
                slidePos = slideMinPos;
                outTake = false;
            }
            if (gamepad2.dpad_down) {
                slidePos = slideHangPos;
            }

            lift.PIDcontrol(slidePos, leftSlide, rightSlide);

            //arm
            if (outTake) {
                armPlace();
                if (gamepad2.a) {
                    clawRotatePlaceDown();
                } else {
                    clawRotatePlace();
                }
                if (gamepad2.x) {
                    outTake = false;
                }
            } else {
                if (gamepad2.x) {
                    clawRotateDown();
                    if (rightArm.getPosition() >= armDownPos) {
                        armVarPos(rightArm.getPosition() + rightArmIncrement);
                    }
                } else if (gamepad2.y) {
                    outTake = true;
                    clawRotatePlace();
                } else {
                    armUp();
                    if (rightArm.getPosition() > 0.28){
                        clawRotateUp();
                    }
                }
            }

            //claw
            if (outTake) {
                if (gamepad2.right_bumper) {
                    leftClawOpen();
                } else {
                    leftClawClosed();
                }
                if (gamepad2.left_bumper) {
                    rightClawOpen();
                } else {
                    rightClawClosed();
                }
            }  else {
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
            }





            telemetry.addData("rightSlide: ", rightSlide.getCurrentPosition());
            telemetry.addData("leftSlide: ", leftSlide.getCurrentPosition());
            telemetry.addData("rightArm: ", rightArm.getPosition());
            telemetry.addData("leftArm: ", leftArm.getPosition());
            telemetry.addData("rightClawRotate: ", rightClawRotate.getPosition());
            telemetry.addData("leftClawRotate: ", leftClawRotate.getPosition());
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
        leftArm.setPosition(armUpPos + armOffset);
        rightArm.setPosition(armUpPos);
    }

    public void armDown() {
        leftArm.setPosition(armDownPos + armOffset);
        rightArm.setPosition(armDownPos);
    }

    public void armPlace() {
        leftArm.setPosition(armPlacePos + armOffset);
        rightArm.setPosition(armPlacePos);
    }

    public void armVarPos(double position) {
        leftArm.setPosition(position + armOffset);
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

    public void clawRotateDown () {
        leftClawRotate.setPosition(clawRotateDownPos);
        rightClawRotate.setPosition(clawRotateDownPos);
    }

    public void clawRotateUp () {
        leftClawRotate.setPosition(clawRotateUpPos);
        rightClawRotate.setPosition(clawRotateUpPos);
    }

    public void clawRotatePlace () {
        leftClawRotate.setPosition(clawRotatePlacePos);
        rightClawRotate.setPosition(clawRotatePlacePos);
    }

    public void clawRotatePlaceDown () {
        leftClawRotate.setPosition(clawRotatePlaceDownPos);
        rightClawRotate.setPosition(clawRotatePlaceDownPos);
    }



}
