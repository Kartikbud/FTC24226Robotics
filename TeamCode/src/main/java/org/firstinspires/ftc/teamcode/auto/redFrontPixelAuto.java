package org.firstinspires.ftc.teamcode.auto;

//import org.firstinspires.ftc.teamcode.auto.RobotFunctions;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "redFrontPixels")
public class redFrontPixelAuto extends LinearOpMode {

    String side = "Center";

    private CameraSubsytem cameraDetection = null;

    boolean togglePreview = true;

    int slidePlacePos = 1450;

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

        SecondSubsytem subsystem = new SecondSubsytem(hardwareMap);

        Pose2d startPose = new Pose2d(12, -60, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        //String side = cameraDetection.elementDetection(telemetry);

        //RED FRONT CENTRE

        TrajectorySequence centreSeq = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> { //initing
                    subsystem.leftClawClosed();
                    subsystem.rightClawClosed();
                    subsystem.armPos(subsystem.armUpPos);
                    subsystem.clawRotatePos(subsystem.clawRotateUpPos);

                })
                .lineToSplineHeading(new Pose2d(27,-25, Math.toRadians(180)))
                .addTemporalMarker(() -> { //placing purple pixel
                    subsystem.clawRotatePos(subsystem.clawRotateDownPos);

                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    subsystem.armPos(subsystem.armDownPos);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    subsystem.rightClawOpen();
                })
                .waitSeconds(1)
                .back(3)
                .addTemporalMarker(() -> { //following purple pixel placement
                    subsystem.armPos(subsystem.armUpPos);
                    subsystem.clawRotatePos(subsystem.clawRotateUpPos);
                    subsystem.rightClawClosed();
                })
                .waitSeconds(1)
                .lineToSplineHeading(new Pose2d(51.5,-35.7, Math.toRadians(180)))
                .addTemporalMarker(() -> { //setting up arm to score
                    subsystem.armPos(subsystem.armPlacePos);
                    subsystem.clawRotatePos(subsystem.clawRotatePlacePos);

                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> { //running slides up
                    subsystem.slidePos(slidePlacePos);
                    subsystem.clawRotatePos(0.95);
                })
                .waitSeconds(3)
                .back(2)
                .addTemporalMarker(() -> {
                    subsystem.leftClawOpen(); //placing yellow pixel
                })
                .waitSeconds(0.5)
                .forward(3)
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    subsystem.leftClawClosed();
                    subsystem.rightClawClosed();
                    subsystem.armPos(subsystem.armUpPos);
                    subsystem.clawRotatePos(subsystem.clawRotateUpPos);
                })
                .addTemporalMarker(() -> {
                    subsystem.slidePos(0); //bringing slides down
                })
                .lineToSplineHeading(new Pose2d(48,-58, Math.toRadians(180)))
                .build();

        //RED FRONT LEFT

        TrajectorySequence leftSeq = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    subsystem.leftClawClosed();
                    subsystem.rightClawClosed();
                    subsystem.armPos(subsystem.armUpPos);
                    subsystem.clawRotatePos(subsystem.clawRotateUpPos);
                })
                .lineToSplineHeading(new Pose2d(7,-34   , Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(9,-31, Math.toRadians(180)))
                .addTemporalMarker(() -> { //placing purple pixel
                    subsystem.clawRotatePos(subsystem.clawRotateDownPos);

                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    subsystem.armPos(subsystem.armDownPos);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    subsystem.rightClawOpen();
                })
                .waitSeconds(1)
                .back(3)
                .addTemporalMarker(() -> { //following purple pixel placement
                    subsystem.armPos(subsystem.armUpPos);
                    subsystem.clawRotatePos(subsystem.clawRotateUpPos);
                    subsystem.rightClawClosed();
                })
                .waitSeconds(1)
                .lineToSplineHeading(new Pose2d(51,-21, Math.toRadians(180)))
                .addTemporalMarker(() -> {
                    subsystem.armPos(subsystem.armPlacePos);
                    subsystem.clawRotatePos(subsystem.clawRotatePlacePos);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    subsystem.slidePos(slidePlacePos);
                    subsystem.clawRotatePos(0.95);
                })
                .waitSeconds(3)
                .back(2)
                .addTemporalMarker(() -> {
                    subsystem.leftClawOpen();
                })
                .waitSeconds(0.5)
                .forward(3)
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    subsystem.leftClawClosed();
                    subsystem.rightClawClosed();
                    subsystem.armPos(subsystem.armUpPos);
                    subsystem.clawRotatePos(subsystem.clawRotateUpPos);
                })
                .addTemporalMarker(() -> {
                    subsystem.slidePos(0);
                })
                .forward(2)
                .lineToSplineHeading(new Pose2d(47,-58, Math.toRadians(180)))
                .build();

        //RED FRONT RIGHT

        TrajectorySequence rightSeq = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    subsystem.leftClawClosed();
                    subsystem.rightClawClosed();
                    subsystem.armPos(subsystem.armUpPos);
                    subsystem.clawRotatePos(subsystem.clawRotateUpPos);
                })
                .lineToSplineHeading(new Pose2d(34,-28, Math.toRadians(180)))
                .addTemporalMarker(() -> { //placing purple pixel
                    subsystem.clawRotatePos(subsystem.clawRotateDownPos);

                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    subsystem.armPos(subsystem.armDownPos);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    subsystem.rightClawOpen();
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> { //following purple pixel placement
                    subsystem.armPos(subsystem.armUpPos);
                    subsystem.clawRotatePos(subsystem.clawRotateUpPos);
                    subsystem.rightClawClosed();
                })
                .lineToSplineHeading(new Pose2d(53,-42, Math.toRadians(180)))
                .addTemporalMarker(() -> {
                    subsystem.armPos(subsystem.armPlacePos);
                    subsystem.clawRotatePos(subsystem.clawRotatePlacePos);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    subsystem.slidePos(slidePlacePos);
                    subsystem.clawRotatePos(0.95);
                })
                .waitSeconds(3)
                .back(2)
                .addTemporalMarker(() -> {
                    subsystem.leftClawOpen();
                })
                .waitSeconds(0.5)
                .forward(3)
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    subsystem.leftClawClosed();
                    subsystem.rightClawClosed();
                    subsystem.armPos(subsystem.armUpPos);
                    subsystem.clawRotatePos(subsystem.clawRotateUpPos);
                })
                .addTemporalMarker(() -> {
                    subsystem.slidePos(0);
                })
                .lineToSplineHeading(new Pose2d(48,-58, Math.toRadians(180)))
                .build();

        /*TrajectorySequence initSeq = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker( () -> {
                    subsystem.rightClawClosed();
                    subsystem.leftClawClosed();
                })
                .waitSeconds(1)
                .addTemporalMarker( () -> {
                    //subsystem.armUp();
                    //left claw close
                })
                .waitSeconds(0.5)
                .forward(14)
                .build();

        TrajectorySequence leftSeq = drive.trajectorySequenceBuilder(initSeq.end())
                .lineToSplineHeading(new Pose2d(18,34, Math.toRadians(0))) //spline to according side
                .addTemporalMarker( () -> {
                    subsystem.armDown();
                    //right claw open
                })
                .waitSeconds(2)
                .addTemporalMarker( () -> {
                    subsystem.armUp();
                    //right claw close
                })
                .forward(30) //adjust depending on location
                .strafeLeft(8) //adjust depending
                .addTemporalMarker( () -> {
                    subsystem.slidePositionTo(500);
                    //left claw open
                })
                .waitSeconds(5)
                .addTemporalMarker( () -> {
                    subsystem.slideDown();
                    //left claw close
                })
                .strafeLeft(15)
                .turn(Math.toRadians(-90))
                        .build();

        TrajectorySequence rightSeq = drive.trajectorySequenceBuilder(initSeq.end())
                .forward(3)
                .lineToSplineHeading(new Pose2d(10,34, Math.toRadians(180)))
                //.strafeRight(8)
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
                //.strafeRight(20)
                .lineToSplineHeading(new Pose2d(49,28, Math.toRadians(0))) //adjust depending on location
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
                })
                .waitSeconds(2)
                .strafeLeft(28)
                .turn(Math.toRadians(-90))
                .addTemporalMarker( () -> {
                    subsystem.armDown();
                })
                .back(2)
                .build();

        //Pose2d startPose = new Pose2d(12, 60, Math.toRadians(270));
        TrajectorySequence centreSeq = drive.trajectorySequenceBuilder(initSeq.end())
                .lineToSplineHeading(new Pose2d(35,21, Math.toRadians(180)))
                //.strafeRight(8)
                .addTemporalMarker( () -> {
                    subsystem.armDown();
                })
                .back(3)
                .waitSeconds(1.5)
                .forward(14)
                .addTemporalMarker( () -> {
                    subsystem.leftClawOpen();
                })
                .waitSeconds(1.5)
                .back(10)
                .waitSeconds(2)
                .addTemporalMarker( () -> {
                    subsystem.armUp();
                    subsystem.leftClawClosed();
                })
                //.strafeRight(20)
                .lineToSplineHeading(new Pose2d(50,34, Math.toRadians(0))) //adjust depending on location
                .addTemporalMarker( () -> {
                    subsystem.slidePositionTo(400);
                })
                .waitSeconds(3)
                .addTemporalMarker( () -> {
                    subsystem.rightClawOpen();
                })
                .waitSeconds(1)
                .back(3)
                .addTemporalMarker( () -> {
                    subsystem.slideDown();
                    subsystem.rightClawClosed();
                })
                .waitSeconds(2)
                .strafeLeft(18)
                .turn(Math.toRadians(-90))
                .addTemporalMarker( () -> {
                    subsystem.armDown();
                })
                .back(2)
                .build();

         leftSeq = drive.trajectorySequenceBuilder(initSeq.end())
                .lineToSplineHeading(new Pose2d(22,48, Math.toRadians(-90))) //spline to according side
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
                .lineToSplineHeading(new Pose2d(50,38, Math.toRadians(0)))
                .addTemporalMarker( () -> {
                    subsystem.slidePositionTo(400);
                })
                .waitSeconds(3)
                .addTemporalMarker( () -> {
                    subsystem.rightClawOpen();
                })
                .waitSeconds(1)
                .back(3)
                .addTemporalMarker( () -> {
                    subsystem.slideDown();
                })
                .waitSeconds(2)
                .strafeLeft(16)
                .turn(Math.toRadians(-90))
                .addTemporalMarker( () -> {
                    subsystem.armDown();
                })
                .back(2)
                .build();

        TrajectorySequence testSeq = drive.trajectorySequenceBuilder(initSeq.end())
                .forward(1)
                .addTemporalMarker( () -> {
                    subsystem.slidePositionTo(400);
                })
                .waitSeconds(5)
                .build();


        //telemetry.update();*/

        while (!opModeIsActive() && !isStopRequested()){
            side = cameraDetection.elementDetection(telemetry);
            telemetry.addData("purple pixel", "red");



            telemetry.update();
        }

        waitForStart();

        if (!isStopRequested()){
            //drive.followTrajectorySequence(initSeq);

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