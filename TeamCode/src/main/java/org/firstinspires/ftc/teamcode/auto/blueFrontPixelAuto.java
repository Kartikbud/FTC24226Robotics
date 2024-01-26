package org.firstinspires.ftc.teamcode.auto;

//import org.firstinspires.ftc.teamcode.auto.RobotFunctions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "blueFrontPixels")
public class blueFrontPixelAuto extends LinearOpMode {

    String side = "Right";

    public int slideDownPos = 0;
    public int slideUpPos = 1600;

    DcMotor leftSlide;
    DcMotor rightSlide;

    Servo leftArm;
    Servo rightArm;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //RobotFunctions robotFunctions = new RobotFunctions();

        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        //robotFunctions.intialize();

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

        //leftArm.setPosition(0);
        //rightArm.setPosition(0);

        Pose2d startPose = new Pose2d(12, 60, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        TrajectorySequence initSeq = drive.trajectorySequenceBuilder(startPose)
                .forward(14)
                .waitSeconds(1) //scan team prop
                .build();

        TrajectorySequence leftSeq = drive.trajectorySequenceBuilder(initSeq.end())
                .lineToSplineHeading(new Pose2d(18,34, Math.toRadians(0))) //spline to according side
                .addTemporalMarker(5, () -> {

                })
                .waitSeconds(1) //drop pixel
                .forward(30) //adjust depending on location
                .strafeLeft(8) //adjust depending
                .waitSeconds(1) //score pixel
                .strafeRight(30)
                .turn(Math.toRadians(-90))
                        .build();

        TrajectorySequence rightSeq = drive.trajectorySequenceBuilder(initSeq.end())
                .lineToSplineHeading(new Pose2d(6,38, Math.toRadians(-135))) //spline to according side\
                .addTemporalMarker(5, () -> {
                    //robotFunctions.slideUp();
                })
                .waitSeconds(1) //drop pixel
                .lineToSplineHeading(new Pose2d(48,28, Math.toRadians(0))) //adjust depending on location
                .waitSeconds(1) //score pixel
                .strafeLeft(30)
                .turn(Math.toRadians(-90))
                .build();

        TrajectorySequence centreSeq = drive.trajectorySequenceBuilder(initSeq.end())
                .lineToSplineHeading(new Pose2d(12,20, Math.toRadians(0)))
                .strafeLeft(4)//spline to according side
                .addTemporalMarker(5, () -> {
                    //testServo.setPosition(1);
                })
                .waitSeconds(1) //drop pixel
                .lineToSplineHeading(new Pose2d(48,35, Math.toRadians(0))) //adjust depending on location
                .waitSeconds(1) //score pixel
                .strafeLeft(22)
                .turn(Math.toRadians(-90))
                .build();

        TrajectorySequence testTraj = drive.trajectorySequenceBuilder(initSeq.end())
                .forward(1)
                .addTemporalMarker(() -> {
                    slideUp(leftSlide, rightSlide);
                    //armUp(leftArm, rightArm);
                })
                .waitSeconds(7)
                .build();

        waitForStart();

        if (!isStopRequested()){
            /*drive.followTrajectorySequence(initSeq);
            if (side.equals("Right")) {
                drive.followTrajectorySequence(rightSeq);
            } else if (side.equals("Centre")) {
                drive.followTrajectorySequence(centreSeq);
            } else {
                drive.followTrajectorySequence(leftSeq);
            }*/

            drive.followTrajectorySequence(testTraj);
        }
    }

    public void armUp(Servo left, Servo right) {
        left.setPosition(0.4);
        right.setPosition(0.4);
    }

    public void armDown(Servo left, Servo right) {
        left.setPosition(0);
        right.setPosition(0);
    }

    public void slideUp(DcMotor left, DcMotor right) {
        left.setTargetPosition(slideUpPos);
        right.setTargetPosition(slideUpPos);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setPower(0.3);
        right.setPower(0.31);
    }

    public void slideDown(DcMotor left,DcMotor right) {
        left.setTargetPosition(slideDownPos);
        right.setTargetPosition(slideDownPos);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setPower(0.3);
        right.setPower(0.3);
    }
}


