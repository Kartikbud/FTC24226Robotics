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

    @Override
    public void runOpMode() throws InterruptedException {
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
            if (teamPosition == "Front") {
                redFront(drive,subsystem);
            } else if (teamPosition == "Back") {
                redBack(drive,subsystem);
            }
        } else if (allianceColour == "Blue") {
            if (teamPosition == "Front") {
                blueFront(drive,subsystem);
            } else if (teamPosition == "Back") {
                blueBack(drive,subsystem);
            }
        }

        waitForStart();

        if (!isStopRequested()){
            //drive.followTrajectorySequence(initSeq);
            /*if (side.equals("Right")) {
                drive.followTrajectorySequence(rightSeq);
            } else if (side.equals("Centre")) {
                drive.followTrajectorySequence(centreSeq);
            } else {
                drive.followTrajectorySequence(leftSeq);
            }*/
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

        TrajectorySequence initBlueFrontSeq = drive.trajectorySequenceBuilder(blueFrontPose)
                .addTemporalMarker( () -> {
                    subsystem.slideDown();
                    subsystem.armUp();
                    //claws closed
                })
                .forward(14)
                .waitSeconds(1) //scan team prop
                .build();

        TrajectorySequence rightBlueFrontSeq = drive.trajectorySequenceBuilder(initBlueFrontSeq.end())
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

        TrajectorySequence centreBlueFrontSeq = drive.trajectorySequenceBuilder(initBlueFrontSeq.end())
                .lineToSplineHeading(new Pose2d(12,20, Math.toRadians(0)))
                .strafeLeft(4)//spline to according side
                .addTemporalMarker( () -> {
                    subsystem.armDown();
                    //right claw open
                })
                .waitSeconds(2)
                .addTemporalMarker( () -> {
                    subsystem.armUp();
                    //right claw close
                })
                .lineToSplineHeading(new Pose2d(48,35, Math.toRadians(0))) //adjust depending on location
                .addTemporalMarker( () -> {
                    subsystem.slidePositionTo(500);
                    //left claw open
                })
                .waitSeconds(5)
                .addTemporalMarker( () -> {
                    subsystem.slideDown();
                    //left claw close
                })
                .strafeLeft(22)
                .turn(Math.toRadians(-90))
                .build();
    }

    public void blueBack(SampleMecanumDrive drive, Subsystem subsystem) {
        drive.setPoseEstimate(blueBackPose);
    }
}


