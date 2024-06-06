package org.firstinspires.ftc.teamcode.auto;

//import org.firstinspires.ftc.teamcode.auto.RobotFunctions;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "blueBackPixels")
public class blueBackPixelAuto extends LinearOpMode {

    String side = "Center";

    boolean yellowBoard = false;
    int slidePos = 1450;

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

        SecondSubsytem subsystem = new SecondSubsytem(hardwareMap);

        Pose2d startPose = new Pose2d(-36, 60, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        //String side = cameraDetection.elementDetection(telemetry);

        //RED FRONT CENTRE

        TrajectorySequence purpleCentreSeq = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    subsystem.leftClawClosed();
                    subsystem.rightClawClosed();
                    subsystem.armPos(subsystem.armUpPos);
                    subsystem.clawRotatePos(subsystem.clawRotateUpPos);
                })
                .lineToSplineHeading(new Pose2d(-41,24, Math.toRadians(0)))
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
                .build();

        TrajectorySequence purpleRightSeq = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    subsystem.leftClawClosed();
                    subsystem.rightClawClosed();
                    subsystem.armPos(subsystem.armUpPos);
                    subsystem.clawRotatePos(subsystem.clawRotateUpPos);
                })
                .lineToSplineHeading(new Pose2d(-58,33.5, Math.toRadians(0)))
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
                .build();

        TrajectorySequence purpleLeftSeq = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    subsystem.leftClawClosed();
                    subsystem.rightClawClosed();
                    subsystem.armPos(subsystem.armUpPos);
                    subsystem.clawRotatePos(subsystem.clawRotateUpPos);
                })
                .lineToSplineHeading(new Pose2d(-40,35, Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(-34,35, Math.toRadians(0)))
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
                .back(5)
                .build();

        TrajectorySequence yellowBoardPlaceCentreSeq = drive.trajectorySequenceBuilder(purpleCentreSeq.end())
                .addTemporalMarker(() -> {
                    subsystem.leftClawClosed();
                    subsystem.rightClawClosed();
                    subsystem.armPos(subsystem.armUpPos);
                    subsystem.clawRotatePos(subsystem.clawRotateUpPos);
                })
                .strafeRight(18)
                .forward(60)
                .turn(Math.toRadians(180))
                .waitSeconds(8)
                .lineToSplineHeading(new Pose2d(52.5,38, Math.toRadians(180)))
                .addTemporalMarker(() -> { //setting up arm to score
                    subsystem.armPos(subsystem.armPlacePos);
                    subsystem.clawRotatePos(subsystem.clawRotatePlacePos);

                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> { //running slides up
                    subsystem.slidePos(slidePos);
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
                .waitSeconds(3)
                .build();

        TrajectorySequence yellowBoardPlaceLeftSeq = drive.trajectorySequenceBuilder(purpleLeftSeq.end())
                .addTemporalMarker(() -> {
                    subsystem.leftClawClosed();
                    subsystem.rightClawClosed();
                    subsystem.armPos(subsystem.armUpPos);
                    subsystem.clawRotatePos(subsystem.clawRotateUpPos);
                })
                .strafeRight(27)
                .forward(60)
                .turn(Math.toRadians(180))
                .waitSeconds(8)
                .lineToSplineHeading(new Pose2d(54,44, Math.toRadians(180)))
                .addTemporalMarker(() -> {
                    subsystem.armPos(subsystem.armPlacePos);
                    subsystem.clawRotatePos(subsystem.clawRotatePlacePos);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    subsystem.slidePos(slidePos);
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
                .waitSeconds(3)
                .build();

        TrajectorySequence yellowBoardPlaceRightSeq = drive.trajectorySequenceBuilder(purpleRightSeq.end())
                .addTemporalMarker(() -> {
                    subsystem.leftClawClosed();
                    subsystem.rightClawClosed();
                    subsystem.armPos(subsystem.armUpPos);
                    subsystem.clawRotatePos(subsystem.clawRotateUpPos);
                })
                .strafeRight(26)
                .forward(72)
                .turn(Math.toRadians(180))
                .waitSeconds(8)
                .lineToSplineHeading(new Pose2d(51,32, Math.toRadians(180)))
                .addTemporalMarker(() -> {
                    subsystem.armPos(subsystem.armPlacePos);
                    subsystem.clawRotatePos(subsystem.clawRotatePlacePos);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    subsystem.slidePos(slidePos);
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
                .waitSeconds(3)
                .build();
/*
        TrajectorySequence centreSeq = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> { //initing
                    subsystem.leftClawClosed();
                    subsystem.rightClawClosed();
                    subsystem.armPos(subsystem.armUpPos);
                    subsystem.clawRotatePos(subsystem.clawRotateUpPos);

                })
                .lineToSplineHeading(new Pose2d(28.75,-26, Math.toRadians(180)))
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
                .lineToSplineHeading(new Pose2d(52.5,-35, Math.toRadians(180)))
                .addTemporalMarker(() -> { //setting up arm to score
                    subsystem.armPos(subsystem.armPlacePos);
                    subsystem.clawRotatePos(subsystem.clawRotatePlacePos);

                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> { //running slides up
                    subsystem.slidePos(subsystem.slideLowPos - 100);
                })
                .waitSeconds(3)
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
                .lineToSplineHeading(new Pose2d(10,-31, Math.toRadians(180)))
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
                .lineToSplineHeading(new Pose2d(51,-22, Math.toRadians(180)))
                .addTemporalMarker(() -> {
                    subsystem.armPos(subsystem.armPlacePos);
                    subsystem.clawRotatePos(subsystem.clawRotatePlacePos);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    subsystem.slidePos(subsystem.slideLowPos-100);
                })
                .waitSeconds(3)
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

        //RED FRONT RIGHT

        TrajectorySequence rightSeq = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    subsystem.leftClawClosed();
                    subsystem.rightClawClosed();
                    subsystem.armPos(subsystem.armUpPos);
                    subsystem.clawRotatePos(subsystem.clawRotateUpPos);
                })
                .lineToSplineHeading(new Pose2d(35,-28, Math.toRadians(180)))
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
                .lineToSplineHeading(new Pose2d(55,-42, Math.toRadians(180)))
                .addTemporalMarker(() -> {
                    subsystem.armPos(subsystem.armPlacePos);
                    subsystem.clawRotatePos(subsystem.clawRotatePlacePos);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    subsystem.slidePos(subsystem.slideLowPos - 100);
                })
                .waitSeconds(3)
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

        */



        while (!opModeIsActive() && !isStopRequested()){
            side = cameraDetection.elementDetection(telemetry);
            telemetry.addData("purple pixel", "right");




            telemetry.update();
        }

        waitForStart();

        if (!isStopRequested()){
            //drive.followTrajectorySequence(initSeq);

            //side = cameraDetection.elementDetection(telemetry);


            if (side.equals("Right")) {
                drive.followTrajectorySequence(purpleRightSeq);
                if (yellowBoard) {
                    drive.followTrajectorySequence(yellowBoardPlaceRightSeq);
                }
            } else if (side.equals("Center")) {
                drive.followTrajectorySequence(purpleCentreSeq);
                if (yellowBoard) {
                    drive.followTrajectorySequence(yellowBoardPlaceCentreSeq);
                }
            } else {
                drive.followTrajectorySequence(purpleLeftSeq);
                if (yellowBoard) {
                    drive.followTrajectorySequence(yellowBoardPlaceLeftSeq);
                }
            }
        }
    }
}


