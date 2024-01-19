package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "redFrontPixels")
public class redFrontPixelAuto extends LinearOpMode {

    String side = "Right";
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(12, -60, Math.toRadians(90));
        Pose2d nextPose = new Pose2d(12, -46, Math.toRadians(90));

        drive.setPoseEstimate(startPose);



        TrajectorySequence initSeq = drive.trajectorySequenceBuilder(startPose)
                .forward(14)
                .waitSeconds(1) //scan team prop
                .build();

        TrajectorySequence rightSeq = drive.trajectorySequenceBuilder(nextPose)
                .lineToSplineHeading(new Pose2d(18,-32, Math.toRadians(0))) //spline to according side
                .waitSeconds(1) //drop pixel
                .forward(30) //adjust depending on location
                .strafeRight(10) //adjust depending
                .waitSeconds(1) //score pixel
                .strafeRight(16)
                .turn(Math.toRadians(90))
                .build();

        TrajectorySequence leftSeq = drive.trajectorySequenceBuilder(nextPose)
                        .build();

        TrajectorySequence centreSeq = drive.trajectorySequenceBuilder(nextPose)
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
