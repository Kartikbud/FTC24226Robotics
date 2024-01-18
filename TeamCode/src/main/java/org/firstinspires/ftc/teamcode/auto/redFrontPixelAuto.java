package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "blueFrontPixels")
public class redFrontPixelAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(12, -60, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .forward(14)
                .waitSeconds(1) //scan team prop
                .splineTo(new Vector2d(15,-34), 0) //spline to according side
                .waitSeconds(1) //drop pixel
                .forward(33) //adjust depending on location
                .strafeRight(8) //adjust depending
                .waitSeconds(1) //score pixel//score pixel
                .build();

        waitForStart();

        if (!isStopRequested()){
            drive.followTrajectorySequence(trajSeq);
        }
    }
}
