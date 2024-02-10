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

    String side = "Center";

    private CameraSubsytem cameraDetection = null;

    boolean togglePreview = true;

    public void HardwareStart() {
        telemetry.addData("Object Creation", "Start");
        telemetry.update();

        cameraDetection = new CameraSubsytem(hardwareMap);

        telemetry.addData("Object Creation", "Done");
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        HardwareStart();

        String curAlliance = "red";

        cameraDetection.setAlliance(curAlliance);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Subsystem subsystem = new Subsystem(hardwareMap);

        Pose2d startPose = new Pose2d(12, 60, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        //String side = cameraDetection.elementDetection(telemetry);

        TrajectorySequence initSeq = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker( () -> {
                    subsystem.rightClawClosed();
                    subsystem.leftClawClosed();
                })
                .waitSeconds(1)
                .addTemporalMarker( () -> {
                    subsystem.armUp();
                    //left claw close
                })
                .waitSeconds(1)
                .forward(14)
                .build();



        TrajectorySequence rightSeq = drive.trajectorySequenceBuilder(initSeq.end())
                .lineToSplineHeading(new Pose2d(10,35, Math.toRadians(180)))
                //.strafeRight(8)
                .addTemporalMarker( () -> {
                    subsystem.armDown();
                })
                .back(3)
                .waitSeconds(1.5)
                .forward(12)
                .addTemporalMarker( () -> {
                    subsystem.leftClawOpen();
                })
                .waitSeconds(1.5)
                .back(10)
                .waitSeconds(2)
                .addTemporalMarker( () -> {
                    subsystem.armUp();
                })
                //.strafeRight(20)
                .lineToSplineHeading(new Pose2d(43,28, Math.toRadians(0))) //adjust depending on location
                .addTemporalMarker( () -> {
                    subsystem.slidePositionTo(1000);
                })
                .waitSeconds(5)
                .addTemporalMarker( () -> {
                    subsystem.rightClawOpen();
                })
                .waitSeconds(1)
                .addTemporalMarker( () -> {
                    subsystem.slideDown();
                })
                .waitSeconds(2)
                .strafeLeft(28)
                .turn(Math.toRadians(-90))
                .build();

        TrajectorySequence centreSeq = drive.trajectorySequenceBuilder(initSeq.end())
                .lineToSplineHeading(new Pose2d(35,21, Math.toRadians(180)))
                //.strafeRight(8)
                .addTemporalMarker( () -> {
                    subsystem.armDown();
                })
                .back(3)
                .waitSeconds(1.5)
                .forward(12)
                .addTemporalMarker( () -> {
                    subsystem.leftClawOpen();
                })
                .waitSeconds(1.5)
                .back(10)
                .waitSeconds(2)
                .addTemporalMarker( () -> {
                    subsystem.armUp();
                })
                //.strafeRight(20)
                .lineToSplineHeading(new Pose2d(46,38, Math.toRadians(0))) //adjust depending on location
                .addTemporalMarker( () -> {
                    subsystem.slidePositionTo(1000);
                })
                .waitSeconds(5)
                .addTemporalMarker( () -> {
                    subsystem.rightClawOpen();
                })
                .waitSeconds(1)
                .addTemporalMarker( () -> {
                    subsystem.slideDown();
                })
                .waitSeconds(2)
                .strafeLeft(18)
                .turn(Math.toRadians(-90))
                .build();

        TrajectorySequence leftSeq = drive.trajectorySequenceBuilder(initSeq.end())
                .lineToSplineHeading(new Pose2d(21,34, Math.toRadians(-90))) //spline to according side
                .addTemporalMarker( () -> {
                    subsystem.armDown();
                })
                .back(3)
                .waitSeconds(1.5)
                .forward(4)
                .addTemporalMarker( () -> {
                    subsystem.leftClawOpen();
                })
                .waitSeconds(1.5)
                .back(10)
                .waitSeconds(2)
                .addTemporalMarker( () -> {
                    subsystem.armUp();
                })
                .lineToSplineHeading(new Pose2d(43,44, Math.toRadians(0)))
                .addTemporalMarker( () -> {
                    subsystem.slidePositionTo(1000);
                })
                .waitSeconds(5)
                .addTemporalMarker( () -> {
                    subsystem.rightClawOpen();
                })
                .waitSeconds(1)
                .addTemporalMarker( () -> {
                    subsystem.slideDown();
                })
                .waitSeconds(2)
                .strafeLeft(18)
                .turn(Math.toRadians(-90))
                .build();

        //telemetry.update();

        while (!opModeIsActive() && !isStopRequested()){
            side = cameraDetection.elementDetection(telemetry);
            //telemetry.addData("color", side);



            telemetry.update();
        }

        waitForStart();

        if (!isStopRequested()){
            drive.followTrajectorySequence(initSeq);

            //side = cameraDetection.elementDetection(telemetry);


            if (side.equals("Right")) {
                drive.followTrajectorySequence(rightSeq);
            } else if (side.equals("Center")) {
                drive.followTrajectorySequence(centreSeq);
            } else {
                drive.followTrajectorySequence(leftSeq);
            }
        }
    }
}


