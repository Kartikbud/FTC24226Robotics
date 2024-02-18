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

@Autonomous(name = "blueBackPixels")
public class blueBackPixels extends LinearOpMode {

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

        String curAlliance = "blue";

        cameraDetection.setAlliance(curAlliance);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Subsystem subsystem = new Subsystem(hardwareMap);

        Pose2d startPose = new Pose2d(-36, 60, Math.toRadians(270));

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
                .lineToSplineHeading(new Pose2d(-46,48, Math.toRadians(-90))) //spline to according side
                .addTemporalMarker( () -> {
                    subsystem.armDown();
                })
                .back(3)
                .waitSeconds(1.5)
                .forward(12)
                .addTemporalMarker( () -> {
                    subsystem.rightClawOpen();
                })
                .waitSeconds(1.5)
                .back(4)
                /*.waitSeconds(2)
                .addTemporalMarker( () -> {
                    subsystem.armUp();
                })
                .forward(40)
                .strafeRight(80)
                .lineToSplineHeading(new Pose2d(49,29, Math.toRadians(0))) //adjust depending on location
                .addTemporalMarker( () -> {
                    subsystem.slidePositionTo(400);
                })
                .waitSeconds(3)
                .addTemporalMarker( () -> {
                    subsystem.leftClawOpen();
                })
                .waitSeconds(1)
                .back(4)
                .addTemporalMarker( () -> {
                    subsystem.slideDown();
                })
                .waitSeconds(2)
                /*.strafeRight(6)
                .turn(Math.toRadians(90))
                .addTemporalMarker( () -> {
                    subsystem.armDown();
                })
                .back(2)*/
                .build();

        TrajectorySequence centreSeq = drive.trajectorySequenceBuilder(initSeq.end())
                .lineToSplineHeading(new Pose2d(-59,25, Math.toRadians(0)))
                //.strafeRight(8)
                .addTemporalMarker( () -> {
                    subsystem.armDown();
                })
                .back(3)
                .waitSeconds(1.5)
                .forward(14)
                .addTemporalMarker( () -> {
                    subsystem.rightClawOpen();
                })
                .waitSeconds(1.5)
                .back(4)
                /*.waitSeconds(2)
                .addTemporalMarker( () -> {
                    subsystem.armUp();
                    subsystem.leftClawClosed();
                })
                .strafeLeft(10)
                .forward(90)
                .lineToSplineHeading(new Pose2d(50,-37, Math.toRadians(0))) //adjust depending on location
                .addTemporalMarker( () -> {
                    subsystem.slidePositionTo(400);
                })
                .waitSeconds(3)
                .addTemporalMarker( () -> {
                    subsystem.rightClawOpen();
                })
                .waitSeconds(1)
                .back(4)
                .addTemporalMarker( () -> {
                    subsystem.slideDown();
                    subsystem.rightClawClosed();
                })
               .waitSeconds(2)
                /*.strafeRight(10)
                .turn(Math.toRadians(90))
                .addTemporalMarker( () -> {
                    subsystem.armDown();
                })
                .back(2)*/
                .build();

        TrajectorySequence leftSeq = drive.trajectorySequenceBuilder(initSeq.end())
                .lineToSplineHeading(new Pose2d(-34,34, Math.toRadians(0)))
                //.strafeRight(8)
                .addTemporalMarker( () -> {
                    subsystem.armDown();
                })
                .back(3)
                .waitSeconds(1.5)
                .forward(4)
                .addTemporalMarker( () -> {
                    subsystem.rightClawOpen();
                })
                .waitSeconds(1.5)
                .back(4)
                /*.waitSeconds(2)
                .addTemporalMarker( () -> {
                    subsystem.armUp();
                })
                .strafeLeft(25)
                .forward(80)
                .lineToSplineHeading(new Pose2d(51,43, Math.toRadians(0)))
                .addTemporalMarker( () -> {
                    subsystem.slidePositionTo(400);
                })
                .waitSeconds(3)
                .addTemporalMarker( () -> {
                    subsystem.leftClawOpen();
                })
                .waitSeconds(1)
                .back(3)
                .addTemporalMarker( () -> {
                    subsystem.slideDown();
                })
                .waitSeconds(2)
                /*.strafeRight(16)
                .turn(Math.toRadians(90))
                .addTemporalMarker( () -> {
                    subsystem.armDown();
                })
                .back(2)*/
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
