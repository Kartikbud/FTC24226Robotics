package org.firstinspires.ftc.teamcode.auto;

//import org.firstinspires.ftc.teamcode.auto.RobotFunctions;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "MainAuto")
public class MainAuto extends LinearOpMode {

    String allianceColour = " ";
    String teamPosition = " ";
    boolean isConfiguring = true;

    String side = "Center";

    Pose2d blueFrontPose = new Pose2d(12, 60, Math.toRadians(270));
    Pose2d blueBackPose = new Pose2d(-36, 60, Math.toRadians(270));
    Pose2d redFrontPose = new Pose2d(12, -60, Math.toRadians(90));
    Pose2d redBackPose = new Pose2d(-36, -60, Math.toRadians(90));

    TrajectorySequence initBlueFrontSeq;
    TrajectorySequence rightBlueFrontSeq;
    TrajectorySequence centreBlueFrontSeq;
    TrajectorySequence leftBlueFrontSeq;

    @Override
    public void runOpMode() throws InterruptedException {
        CameraSubsytem cameraDetection = new CameraSubsytem(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Subsystem subsystem = new Subsystem(hardwareMap);

        while (isConfiguring) {
            if (gamepad1.x) {
                allianceColour = "Blue";
            }
            if (gamepad1.b) {
                allianceColour = "Red";
            }
            if (gamepad1.y) {
                teamPosition = "Front";
            }
            if (gamepad1.a) {
                teamPosition = "Back";
            }
            if (gamepad1.left_stick_button) {
                isConfiguring = false;
            }
            telemetry.addLine( "ALLIANCE COLOUR:");
            telemetry.addLine("Red (Press B)");
            telemetry.addLine("Blue (Press X)");
            telemetry.addLine( "TEAM POSITION:");
            telemetry.addLine("Front (Press Y)");
            telemetry.addLine("Back (Press A)");
            telemetry.addLine("PRESS LEFT STICK TO CONFIRM");
            telemetry.addLine("Selected Alliance Colour: " + allianceColour);
            telemetry.addLine("Selected Team Position: " + teamPosition);
            telemetry.update();
        }

        telemetry.addLine("Initialized");
        telemetry.addLine("Alliance Colour: " + allianceColour);
        telemetry.addLine("Team Position: " + teamPosition);
        telemetry.update();

        if (allianceColour == "Red") {
            cameraDetection.setAlliance("red");
            if (teamPosition == "Front") {
                redFront(drive,subsystem);
            } else if (teamPosition == "Back") {
                redBack(drive,subsystem);
            }
        } else if (allianceColour == "Blue") {
            cameraDetection.setAlliance("blue");
            if (teamPosition == "Front") {
                blueFront(drive,subsystem);
            } else if (teamPosition == "Back") {
                blueBack(drive,subsystem);
            }
        }

        while (!opModeIsActive() && !isStopRequested()){
            side = cameraDetection.elementDetection(telemetry);
            telemetry.addData("color", side);
            telemetry.update();
        }

        waitForStart();

        if (!isStopRequested()){
            if (allianceColour == "Blue" && teamPosition == "Front") {
                drive.followTrajectorySequence(initBlueFrontSeq);
                if (side.equals("Right")) {
                    drive.followTrajectorySequence(rightBlueFrontSeq);
                } else if (side.equals("Center")) {
                    drive.followTrajectorySequence(centreBlueFrontSeq);
                } else {
                    drive.followTrajectorySequence(leftBlueFrontSeq);
                }
            } else if (allianceColour == "Blue" && teamPosition == "Back") {

            } else if (allianceColour == "Red" && teamPosition == "Front") {

            } else if (allianceColour == "Red" && teamPosition == "Back") {

            }
        }
    }

    public void redFront(SampleMecanumDrive drive, Subsystem subsystem) {
        drive.setPoseEstimate(redFrontPose);





    }

    public void redBack(SampleMecanumDrive drive, Subsystem subsystem) {
        drive.setPoseEstimate(redBackPose);
    }

    public void blueFront(SampleMecanumDrive drive, Subsystem subsystem) {
        drive.setPoseEstimate(blueFrontPose);

        initBlueFrontSeq = drive.trajectorySequenceBuilder(blueFrontPose)
                .addTemporalMarker( () -> {
                    subsystem.rightClawClosed();
                    subsystem.leftClawClosed();
                    subsystem.slideDown();
                    subsystem.armUp();
                })
                .forward(14)
                .build();


        //NOT DONE
        rightBlueFrontSeq = drive.trajectorySequenceBuilder(initBlueFrontSeq.end())
                .lineToSplineHeading(new Pose2d(6,38, Math.toRadians(-135))) //spline to according side\
                .addTemporalMarker( () -> {
                    subsystem.armDown();
                    //right claw open
                })
                .waitSeconds(2)
                .addTemporalMarker( () -> {
                    subsystem.armUp();
                    //right claw close
                })
                .lineToSplineHeading(new Pose2d(48,28, Math.toRadians(0))) //adjust depending on location
                .addTemporalMarker( () -> {
                    subsystem.slidePositionTo(500);
                    //left claw open
                })
                .waitSeconds(5)
                .addTemporalMarker( () -> {
                    subsystem.slideDown();
                    //left claw close
                })
                .strafeLeft(29)
                .turn(Math.toRadians(-90))
                .build();


        centreBlueFrontSeq = drive.trajectorySequenceBuilder(initBlueFrontSeq.end())
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

        leftBlueFrontSeq = drive.trajectorySequenceBuilder(initBlueFrontSeq.end())
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
    }

    public void blueBack(SampleMecanumDrive drive, Subsystem subsystem) {
        drive.setPoseEstimate(blueBackPose);
    }
}


