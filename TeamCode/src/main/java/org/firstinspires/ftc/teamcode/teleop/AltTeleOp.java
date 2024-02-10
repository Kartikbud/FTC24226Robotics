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

@TeleOp(name = "AltTeleOp")
public class AltTeleOp extends LinearOpMode {

    DcMotor rightFront, rightRear, leftFront, leftRear;
    DcMotor leftSlide, rightSlide;
    DcMotor leftLift, rightLift;
    Servo leftArm, rightArm;
    Servo leftClaw, rightClaw;
    Servo drone;
    IMU imu;
    Orientation myRobotOrientation;
    double axial_drive;
    double lateral_drive;
    double yaw_drive;
    double heading_drive;
    double input_lift;
    double liftDistance;


    //constants
    double DRIVE_POWER_SCALE = 0.9;
    double SLIDE_POWER_SCALE = 0.62;
    double LIFT_POWER_SCALE = 1;
    double FINE_DRIVE_POWER_SCALE = DRIVE_POWER_SCALE/3;
    int liftDownPos = 0;
    double armUpPos = 0.75;
    double armDownPos = 0.02;
    double liftCountPerRev = 1440;
    double slideCountPerRev = 383.6;
    double diameterLift = 0.88;
    double diameterSlide = 1.4;
    double liftCountPerInch = liftCountPerRev / (diameterLift * Math.PI);
    double slideCountPerInch = slideCountPerRev / (diameterSlide + Math.PI);
    double slideMaxDistance = 20.3418124319;
    double slideMinDistance = 0;
    double clawClosedPos = 1;
    double clawOpenPos = 0.85;
    double droneLockPos = 1;
    double droneUnlockPos = 0;
    boolean slideLiftSyncOn = true;


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
        leftSlide.setDirection(DcMotor.Direction.FORWARD);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //lift initialization
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        leftLift.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setTargetPosition(liftDownPos);
        rightLift.setTargetPosition(liftDownPos);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

        //drone initialization
        drone = hardwareMap.get(Servo.class, "drone");
        drone.setDirection(Servo.Direction.FORWARD);

        //imu initialization
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            axial_drive = -gamepad1.left_stick_y;
            lateral_drive = gamepad1.left_stick_x;
            yaw_drive = gamepad1.right_stick_x;
            heading_drive = myRobotOrientation.firstAngle;

            input_lift = gamepad2.left_stick_y;

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

            if (gamepad1.a) {
                driveCorrection(myRobotOrientation.firstAngle);
            }

            if (gamepad1.b) {

            }

            if (gamepad2.dpad_right) {
                slideLiftSyncOn = false;
            } else if (gamepad2.dpad_left) {
                slideLiftSyncOn = true;
            }

            if (slideLiftSyncOn) {
                if (gamepad2.b) {
                    slideLift(slideMaxDistance);
                }

                if (gamepad2.left_stick_button) {
                    slideLift(slideMinDistance);
                }

                if (gamepad1.left_stick_button) {
                    slideLift(slideMinDistance);
                }

                if (gamepad2.a) {
                    slideLift(slideMaxDistance/2);
                }
            } else {
                lift(input_lift);

                if (gamepad2.b) {
                    slide(slideMaxDistance);
                }

                if (gamepad2.left_stick_button) {
                    slide(slideMinDistance);
                }

                if (gamepad1.left_stick_button) {
                    slide(slideMinDistance);
                }

                if (gamepad2.a) {
                    slide(slideMaxDistance/2);
                }
            }

            if (gamepad2.x) {
                armDown();
            } else {
                armUp();
            }

            if (gamepad1.y) {
                droneUnlock();
            } else {
                droneLock();
            }

            if (gamepad2.right_bumper) {
                rightClaw(clawOpenPos);
            } else {
                rightClaw(clawClosedPos);
            }

            if (gamepad2.left_bumper) {
                leftClaw(clawOpenPos);
            } else {
                leftClaw(clawClosedPos);
            }

            //mecanum_drive_field(axial_drive,lateral_drive,yaw_drive,heading_drive);
            mecanum_drive_robot(axial_drive,lateral_drive,yaw_drive);


            telemetry.addData("right", rightArm.getPosition());
            telemetry.addData("left", leftArm.getPosition());
            telemetry.update();

        }
    }
    public void mecanum_drive_field(double axial, double lateral, double yaw, double heading) {
        // Rotate the movement direction counter to the bot's rotation
        double rotX = lateral * Math.cos(-heading) - axial * Math.sin(-heading);
        double rotY = lateral * Math.sin(-heading) + axial * Math.cos(-heading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(yaw), 1);
        double leftFrontPower = (rotY + rotX + yaw) / denominator;
        double leftRearPower = (rotY - rotX + yaw) / denominator;
        double rightFrontPower = (rotY - rotX - yaw) / denominator;
        double rightRearPower = (rotY + rotX - yaw) / denominator;
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

    public void slideLift(double distance) {
        if ((slideCountPerInch * distance) > leftSlide.getCurrentPosition()) {
            liftDistance = distance - 4;
        } else {
            liftDistance = distance;
        }

        leftSlide.setTargetPosition((int) (slideCountPerInch * distance));
        rightSlide.setTargetPosition((int) (slideCountPerInch * distance));
        leftLift.setTargetPosition((int) (liftCountPerInch * liftDistance));
        rightLift.setTargetPosition((int) (liftCountPerInch * liftDistance));
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setMode((DcMotor.RunMode.RUN_TO_POSITION));
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(SLIDE_POWER_SCALE);
        rightSlide.setPower(SLIDE_POWER_SCALE);
        leftLift.setPower(SLIDE_POWER_SCALE/0.64);
        rightLift.setPower(SLIDE_POWER_SCALE/0.64);
    }

    public void slide(double distance) {
        leftSlide.setTargetPosition((int) (slideCountPerInch * distance));
        rightSlide.setTargetPosition((int) (slideCountPerInch * distance));
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
        //rightArm.setPosition(Range.clip(0, 1, rightArm.getPosition() + .01));
        //leftArm.setPosition(Range.clip(0, 1, leftArm.getPosition() + .01));

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

    public void lift(double power) {
        if (power > 0) {
            leftLift.setTargetPosition(leftLift.getCurrentPosition() + 200);
            rightLift.setTargetPosition(rightLift.getCurrentPosition() + 200);
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftLift.setPower(LIFT_POWER_SCALE);
            rightLift.setPower(LIFT_POWER_SCALE);
        } else if (power < 0) {
            leftLift.setTargetPosition(leftLift.getCurrentPosition() - 200);
            rightLift.setTargetPosition(rightLift.getCurrentPosition() - 200);
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftLift.setPower(-LIFT_POWER_SCALE);
            rightLift.setPower(-LIFT_POWER_SCALE);
        } else {
            leftLift.setTargetPosition(leftLift.getCurrentPosition());
            rightLift.setTargetPosition(rightLift.getCurrentPosition());
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftLift.setPower(LIFT_POWER_SCALE);
            rightLift.setPower(LIFT_POWER_SCALE);
        }
    }

    public void rightClaw (double position) {
        rightClaw.setPosition(position);
    }

    public void leftClaw (double position) {
        leftClaw.setPosition(position);
    }

    public void driveCorrection(double heading) {
        if ((-45 <= heading) && (heading <= 45)) {
            if (-45 <= heading && heading < 0) {
                yaw_drive = 0.3;
                if (heading >= -1 && heading <= 1){
                    return;
                }
            } else if (heading <= 45) {
                yaw_drive = -0.3;
                if (heading >= -1 && heading <= 1){
                    return;
                }
            }
        }




    }

    /*public void flipTurn(double heading) {
        if
    }*/

}