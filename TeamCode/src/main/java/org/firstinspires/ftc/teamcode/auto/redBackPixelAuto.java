package org.firstinspires.ftc.teamcode.auto;

//import org.firstinspires.ftc.teamcode.auto.RobotFunctions;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "redBackPixels")
public class redBackPixelAuto extends LinearOpMode {

    String side = "Center";

    boolean yellowBoard = false;

    boolean yellowPark = true;

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

        String curAlliance = "red";

        cameraDetection.setAlliance(curAlliance);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        SecondSubsytem subsystem = new SecondSubsytem(hardwareMap);

        Pose2d startPose = new Pose2d(-36, -60, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        //String side = cameraDetection.elementDetection(telemetry);

        //BLUE FRONT CENTRE

        TrajectorySequence purpleCentreSeq = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    subsystem.leftClawClosed();
                    subsystem.rightClawClosed();
                    subsystem.armPos(subsystem.armUpPos);
                    subsystem.clawRotatePos(subsystem.clawRotateUpPos);
                })
                .lineToSplineHeading(new Pose2d(-41,-23.5, Math.toRadians(0)))
                .addTemporalMarker(() -> { //placing purple pixel
                    subsystem.clawRotatePos(subsystem.clawRotateDownPos);

                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    subsystem.armPos(subsystem.armDownPos);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    subsystem.leftClawOpen();
                })
                .waitSeconds(1)
                .back(3)
                .addTemporalMarker(() -> {
                    subsystem.leftClawClosed();
                    subsystem.rightClawClosed();
                    subsystem.armPos(subsystem.armUpPos);
                    subsystem.clawRotatePos(subsystem.clawRotateUpPos);
                })
                .build();

        TrajectorySequence purpleLeftSeq = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    subsystem.leftClawClosed();
                    subsystem.rightClawClosed();
                    subsystem.armPos(subsystem.armUpPos);
                    subsystem.clawRotatePos(subsystem.clawRotateUpPos);
                })
                .lineToSplineHeading(new Pose2d(-56,-26, Math.toRadians(0)))
                .addTemporalMarker(() -> { //placing purple pixel
                    subsystem.clawRotatePos(subsystem.clawRotateDownPos);

                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    subsystem.armPos(subsystem.armDownPos);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    subsystem.leftClawOpen();
                })
                .waitSeconds(1)
                .back(2)
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    //subsystem.leftClawClosed();
                    subsystem.rightClawClosed();
                    subsystem.armPos(subsystem.armUpPos);
                    subsystem.clawRotatePos(subsystem.clawRotateUpPos);
                })
                .build();

        TrajectorySequence purpleRightSeq = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    subsystem.leftClawClosed();
                    subsystem.rightClawClosed();
                    subsystem.armPos(subsystem.armUpPos);
                    subsystem.clawRotatePos(subsystem.clawRotateUpPos);
                })
                .lineToSplineHeading(new Pose2d(-40,-33, Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(-34,-36, Math.toRadians(0)))
                .addTemporalMarker(() -> { //placing purple pixel
                    subsystem.clawRotatePos(subsystem.clawRotateDownPos);

                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    subsystem.armPos(subsystem.armDownPos);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    subsystem.leftClawOpen();
                })
                .waitSeconds(1)
                .back(3)
                .addTemporalMarker(() -> {
                    subsystem.leftClawClosed();
                    subsystem.rightClawClosed();
                    subsystem.armPos(subsystem.armUpPos);
                    subsystem.clawRotatePos(subsystem.clawRotateUpPos);
                })
                .build();

        TrajectorySequence yellowBoardPlaceCentreSeq = drive.trajectorySequenceBuilder(purpleCentreSeq.end())
                .addTemporalMarker(() -> {
                    subsystem.leftClawClosed();
                    subsystem.rightClawClosed();
                    subsystem.armPos(subsystem.armUpPos);
                    subsystem.clawRotatePos(subsystem.clawRotateUpPos);
                })
                .strafeLeft(19)
                .forward(60)
                .turn(Math.toRadians(180))
                .waitSeconds(5)
                .lineToSplineHeading(new Pose2d(52,-33, Math.toRadians(180)))
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
                    subsystem.rightClawOpen();
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
                .strafeLeft(28)
                .forward(60)
                .turn(Math.toRadians(180))
                .waitSeconds(5)
                .lineToSplineHeading(new Pose2d(48,-41.5, Math.toRadians(180)))
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
                    subsystem.rightClawOpen();
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

        TrajectorySequence yellowBoardPlaceLeftSeq = drive.trajectorySequenceBuilder(purpleLeftSeq.end())
                .addTemporalMarker(() -> {
                    subsystem.leftClawClosed();
                    subsystem.rightClawClosed();
                    subsystem.armPos(subsystem.armUpPos);
                    subsystem.clawRotatePos(subsystem.clawRotateUpPos);
                })
                .strafeLeft(24)
                .forward(80)
                .turn(Math.toRadians(180))
                .waitSeconds(5)
                .lineToSplineHeading(new Pose2d(52,-24, Math.toRadians(180)))
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
                    subsystem.rightClawOpen();
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

        TrajectorySequence yellowParkLeftSeq = drive.trajectorySequenceBuilder(purpleLeftSeq.end())
                .addTemporalMarker(() -> {
                    subsystem.leftClawClosed();
                    subsystem.rightClawClosed();
                    subsystem.armPos(subsystem.armUpPos);
                    subsystem.clawRotatePos(subsystem.clawRotateUpPos);
                })
                .strafeLeft(25)
                .waitSeconds(10)
                .forward(115)
                /*.addTemporalMarker(() -> {
                    subsystem.armPos(subsystem.armDownPos);
                    subsystem.clawRotatePos(subsystem.clawRotateDownPos);
                })*/
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    subsystem.rightClawOpen();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    subsystem.leftClawClosed();
                    subsystem.rightClawClosed();
                    subsystem.armPos(subsystem.armUpPos);
                    subsystem.clawRotatePos(subsystem.clawRotateUpPos);
                })
                .build();

        TrajectorySequence yellowParkRightSeq = drive.trajectorySequenceBuilder(purpleRightSeq.end())
                .addTemporalMarker(() -> {
                    subsystem.leftClawClosed();
                    subsystem.rightClawClosed();
                    subsystem.armPos(subsystem.armUpPos);
                    subsystem.clawRotatePos(subsystem.clawRotateUpPos);
                })
                .strafeLeft(28)
                .waitSeconds(10)
                .forward(90)
                /*.addTemporalMarker(() -> {
                    subsystem.armPos(subsystem.armDownPos);
                    subsystem.clawRotatePos(subsystem.clawRotateDownPos);
                })*/
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    subsystem.rightClawOpen();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    subsystem.leftClawClosed();
                    subsystem.rightClawClosed();
                    subsystem.armPos(subsystem.armUpPos);
                    subsystem.clawRotatePos(subsystem.clawRotateUpPos);
                })
                .build();

        TrajectorySequence yellowParkCentreSeq = drive.trajectorySequenceBuilder(purpleCentreSeq.end())
                .addTemporalMarker(() -> {
                    subsystem.leftClawClosed();
                    subsystem.rightClawClosed();
                    subsystem.armPos(subsystem.armUpPos);
                    subsystem.clawRotatePos(subsystem.clawRotateUpPos);
                })
                .strafeLeft(19)
                .waitSeconds(10)
                .forward(100)
                /*.addTemporalMarker(() -> {
                    //subsystem.armPos(subsystem.armDownPos);
                    //subsystem.clawRotatePos(subsystem.clawRotateDownPos);
                })*/
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    subsystem.rightClawOpen();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    subsystem.leftClawClosed();
                    subsystem.rightClawClosed();
                    subsystem.armPos(subsystem.armUpPos);
                    subsystem.clawRotatePos(subsystem.clawRotateUpPos);
                })
                .build();

        /*TrajectorySequence centreSeq = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> { //initing
                    subsystem.leftClawClosed();
                    subsystem.rightClawClosed();
                    subsystem.armPos(subsystem.armUpPos);
                    subsystem.clawRotatePos(subsystem.clawRotateUpPos);

                })
                .lineToSplineHeading(new Pose2d(28,21, Math.toRadians(180)))
                .addTemporalMarker(() -> { //placing purple pixel
                    subsystem.clawRotatePos(subsystem.clawRotateDownPos);

                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    subsystem.armPos(subsystem.armDownPos);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    subsystem.leftClawOpen();
                })
                .waitSeconds(1)
                .back(3)
                .addTemporalMarker(() -> { //following purple pixel placement
                    subsystem.armPos(subsystem.armUpPos);
                    subsystem.clawRotatePos(subsystem.clawRotateUpPos);
                    subsystem.leftClawClosed();
                })
                .waitSeconds(1)
                .lineToSplineHeading(new Pose2d(54,31, Math.toRadians(180)))
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
                    subsystem.rightClawOpen(); //placing yellow pixel
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
                .lineToSplineHeading(new Pose2d(48,58, Math.toRadians(180)))
                .build();

        //BLUE FRONT RIGHT

        TrajectorySequence rightSeq = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    subsystem.leftClawClosed();
                    subsystem.rightClawClosed();
                    subsystem.armPos(subsystem.armUpPos);
                    subsystem.clawRotatePos(subsystem.clawRotateUpPos);
                })
                .lineToSplineHeading(new Pose2d(18,34, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(9,31, Math.toRadians(180)))
                .addTemporalMarker(() -> { //placing purple pixel
                    subsystem.clawRotatePos(subsystem.clawRotateDownPos);

                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    subsystem.armPos(subsystem.armDownPos);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    subsystem.leftClawOpen();
                })
                .waitSeconds(1)
                .back(3)
                .addTemporalMarker(() -> { //following purple pixel placement
                    subsystem.armPos(subsystem.armUpPos);
                    subsystem.clawRotatePos(subsystem.clawRotateUpPos);
                    subsystem.leftClawClosed();
                })
                .waitSeconds(1)
                .lineToSplineHeading(new Pose2d(49,20.5, Math.toRadians(180)))
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
                    subsystem.rightClawOpen();
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
                .lineToSplineHeading(new Pose2d(48,58, Math.toRadians(180)))
                .build();

        //BLUE FRONT LEFT

        TrajectorySequence leftSeq = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    subsystem.leftClawClosed();
                    subsystem.rightClawClosed();
                    subsystem.armPos(subsystem.armUpPos);
                    subsystem.clawRotatePos(subsystem.clawRotateUpPos);
                })
                .lineToSplineHeading(new Pose2d(35,28, Math.toRadians(180)))
                .addTemporalMarker(() -> { //placing purple pixel
                    subsystem.clawRotatePos(subsystem.clawRotateDownPos);

                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    subsystem.armPos(subsystem.armDownPos);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    subsystem.leftClawOpen();
                })
                .waitSeconds(1)
                .lineToSplineHeading(new Pose2d(56,34, Math.toRadians(180)))
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
                    subsystem.rightClawOpen();
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
                .lineToSplineHeading(new Pose2d(48,58, Math.toRadians(180)))
                .build(); */


        while (!opModeIsActive() && !isStopRequested()){
            side = cameraDetection.elementDetection(telemetry);
            telemetry.addData("purple pixel", "left");




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
                if (yellowPark) {
                    drive.followTrajectorySequence(yellowParkRightSeq);
                }
            } else if (side.equals("Center")) {
                drive.followTrajectorySequence(purpleCentreSeq);
                if (yellowBoard) {
                    drive.followTrajectorySequence(yellowBoardPlaceCentreSeq);
                }
                if (yellowPark) {
                    drive.followTrajectorySequence(yellowParkCentreSeq);
                }
            } else {
                drive.followTrajectorySequence(purpleLeftSeq);
                if (yellowBoard) {
                    drive.followTrajectorySequence(yellowBoardPlaceLeftSeq);
                }
                if (yellowPark) {
                    drive.followTrajectorySequence(yellowParkLeftSeq);
                }
            }
        }
    }
}


