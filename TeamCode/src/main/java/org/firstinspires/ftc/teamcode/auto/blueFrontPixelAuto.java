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

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Subsystem subsystem = new Subsystem(hardwareMap);

        Pose2d startPose = new Pose2d(12, 60, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        TrajectorySequence initSeq = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker( () -> {
                    subsystem.slideDown();
                    subsystem.armUp();
                    //claws closed
                })
                .forward(14)
                .waitSeconds(1) //scan team prop
                .build();

        TrajectorySequence leftSeq = drive.trajectorySequenceBuilder(initSeq.end())
                .lineToSplineHeading(new Pose2d(18,34, Math.toRadians(0))) //spline to according side
                .addTemporalMarker( () -> {
                    subsystem.armDown();
                    //right claw open
                })
                .waitSeconds(0.5)
                .addTemporalMarker( () -> {
                    subsystem.armUp();
                    //right claw close
                })
                .forward(30) //adjust depending on location
                .strafeLeft(8) //adjust depending
                .addTemporalMarker( () -> {
                    subsystem.slidePositionTo(100);
                    //left claw open
                })
                .waitSeconds(0.5)
                .addTemporalMarker( () -> {
                    subsystem.slideDown();
                    //left claw close
                })
                .strafeLeft(15)
                .turn(Math.toRadians(-90))
                        .build();

        TrajectorySequence rightSeq = drive.trajectorySequenceBuilder(initSeq.end())
                .lineToSplineHeading(new Pose2d(6,38, Math.toRadians(-135))) //spline to according side\
                .addTemporalMarker( () -> {
                    subsystem.armDown();
                    //right claw open
                })
                .waitSeconds(0.5)
                .addTemporalMarker( () -> {
                    subsystem.armUp();
                    //right claw close
                })
                .lineToSplineHeading(new Pose2d(48,28, Math.toRadians(0))) //adjust depending on location
                .addTemporalMarker( () -> {
                    subsystem.slidePositionTo(100);
                    //left claw open
                })
                .waitSeconds(0.5)
                .addTemporalMarker( () -> {
                    subsystem.slideDown();
                    //left claw close
                })
                .strafeLeft(29)
                .turn(Math.toRadians(-90))
                .build();

        TrajectorySequence centreSeq = drive.trajectorySequenceBuilder(initSeq.end())
                .lineToSplineHeading(new Pose2d(12,20, Math.toRadians(0)))
                .strafeLeft(4)//spline to according side
                .addTemporalMarker( () -> {
                    subsystem.armDown();
                    //right claw open
                })
                .waitSeconds(0.5)
                .addTemporalMarker( () -> {
                    subsystem.armUp();
                    //right claw close
                })
                .lineToSplineHeading(new Pose2d(48,35, Math.toRadians(0))) //adjust depending on location
                .addTemporalMarker( () -> {
                    subsystem.slidePositionTo(100);
                    //left claw open
                })
                .waitSeconds(0.5)
                .addTemporalMarker( () -> {
                    subsystem.slideDown();
                    //left claw close
                })
                .strafeLeft(22)
                .turn(Math.toRadians(-90))
                .build();

        waitForStart();

        if (!isStopRequested()){
            drive.followTrajectorySequence(initSeq);
            if (side.equals("Right")) {
                drive.followTrajectorySequence(rightSeq);
            } else if (side.equals("Centre")) {
                drive.followTrajectorySequence(centreSeq);
            } else {
                drive.followTrajectorySequence(leftSeq);
            }
        }
    }
}


