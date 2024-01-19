package org.firstinspires.ftc.teamcode.auto;

//import org.firstinspires.ftc.teamcode.auto.RobotFunctions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "blueFrontPixels")
public class blueFrontPixelAuto extends LinearOpMode {
    RobotFunctions robotFunctions = new RobotFunctions();
    String side = "Right";

    Servo testServo;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        robotFunctions.testServo = hardwareMap.get(Servo.class, "servo");

        Pose2d startPose = new Pose2d(12, 60, Math.toRadians(270));
        Pose2d initPose = new Pose2d(12, 46, Math.toRadians(270));

        //component inits

        //testServo = hardwareMap.get(Servo.class, "servo");



        drive.setPoseEstimate(startPose);

        TrajectorySequence initSeq = drive.trajectorySequenceBuilder(startPose)
                .forward(14)
                .waitSeconds(1) //scan team prop
                .build();

        TrajectorySequence leftSeq = drive.trajectorySequenceBuilder(initPose)
                .lineToSplineHeading(new Pose2d(18,34, Math.toRadians(0))) //spline to according side
                .addTemporalMarker(5, () -> {
                    //testServo.setPosition(1);
                })
                .waitSeconds(1) //drop pixel
                .forward(30) //adjust depending on location
                .strafeLeft(8) //adjust depending
                .waitSeconds(1) //score pixel
                .strafeRight(30)
                .turn(Math.toRadians(-90))
                        .build();

        TrajectorySequence rightSeq = drive.trajectorySequenceBuilder(initPose)
                .lineToSplineHeading(new Pose2d(6,38, Math.toRadians(-135))) //spline to according side\
                .addTemporalMarker(5, () -> {
                    //testServo.setPosition(1);
                })
                .waitSeconds(1) //drop pixel
                .lineToSplineHeading(new Pose2d(48,28, Math.toRadians(0))) //adjust depending on location
                .waitSeconds(1) //score pixel
                .strafeLeft(30)
                .turn(Math.toRadians(-90))
                .build();

        TrajectorySequence centreSeq = drive.trajectorySequenceBuilder(initPose)
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

        TrajectorySequence testTraj = drive.trajectorySequenceBuilder(initPose)
                .forward(5)
                .addTemporalMarker(5, () -> {
                    robotFunctions.servo();
                }).build();

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

    public void clawServo (Servo servo) {
        servo.setPosition(0.2);
    }
}