package org.firstinspires.ftc.teamcode.auto;

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
    int waitBuffer = 0;
    boolean isConfiguring = true;

    String side = "Center";

    Pose2d blueFrontPose = new Pose2d(12, 60, Math.toRadians(270));
    Pose2d blueBackPose = new Pose2d(-36, 60, Math.toRadians(270));
    Pose2d redFrontPose = new Pose2d(12, -60, Math.toRadians(90));
    Pose2d redBackPose = new Pose2d(-36, -60, Math.toRadians(90));

    TrajectorySequence initBlueFrontSeq, rightBlueFrontSeq, centreBlueFrontSeq, leftBlueFrontSeq;
    TrajectorySequence initRedFrontSeq, rightRedFrontSeq, centreRedFrontSeq, leftRedFrontSeq;
    TrajectorySequence initRedBackSeq, rightRedBackSeq, centreRedBackSeq, leftRedBackSeq;
    TrajectorySequence initBlueBackSeq, rightBlueBackSeq, centreBlueBackSeq, leftBlueBackSeq;

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
            if (gamepad1.dpad_up) {
                waitBuffer += 1;
            }
            if (gamepad1.dpad_down) {
                if (waitBuffer > 0) {
                    waitBuffer -= 1;
                }
            }
            telemetry.addLine( "ALLIANCE COLOUR:");
            telemetry.addLine("Red (Press B)");
            telemetry.addLine("Blue (Press X)");
            telemetry.addLine( "TEAM POSITION:");
            telemetry.addLine("Front (Press Y)");
            telemetry.addLine("Back (Press A)");
            telemetry.addLine( "WAIT BUFFER:");
            telemetry.addLine("Increase (Press Dpad Up)");
            telemetry.addLine("Decrease (Press Dpad Down)");
            telemetry.addLine("PRESS LEFT STICK TO CONFIRM");
            telemetry.addLine("Selected Alliance Colour: " + allianceColour);
            telemetry.addLine("Selected Team Position: " + teamPosition);
            telemetry.addLine("Selected Wait Buffer: " + waitBuffer);
            telemetry.update();
        }

        telemetry.addLine("Initialized");
        telemetry.addLine("Alliance Colour: " + allianceColour);
        telemetry.addLine("Team Position: " + teamPosition);
        telemetry.addLine("Wait Buffer: " + waitBuffer);
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
                drive.followTrajectorySequence(initBlueBackSeq);
                if (side.equals("Right")) {
                    drive.followTrajectorySequence(rightBlueBackSeq);
                } else if (side.equals("Center")) {
                    drive.followTrajectorySequence(centreBlueBackSeq);
                } else {
                    drive.followTrajectorySequence(leftBlueBackSeq);
                }

            } else if (allianceColour == "Red" && teamPosition == "Front") {
                drive.followTrajectorySequence(initRedFrontSeq);
                if (side.equals("Right")) {
                    drive.followTrajectorySequence(rightRedFrontSeq);
                } else if (side.equals("Center")) {
                    drive.followTrajectorySequence(centreRedFrontSeq);
                } else {
                    drive.followTrajectorySequence(leftRedFrontSeq);
                }

            } else if (allianceColour == "Red" && teamPosition == "Back") {
                drive.followTrajectorySequence(initRedBackSeq);
                if (side.equals("Right")) {
                    drive.followTrajectorySequence(rightRedBackSeq);
                } else if (side.equals("Center")) {
                    drive.followTrajectorySequence(centreRedBackSeq);
                } else {
                    drive.followTrajectorySequence(leftRedBackSeq);
                }
            }
        }
    }

    public void redFront(SampleMecanumDrive drive, Subsystem subsystem) {
        drive.setPoseEstimate(redFrontPose);

        initRedFrontSeq = drive.trajectorySequenceBuilder(redFrontPose)
                .addTemporalMarker( () -> {
                    subsystem.rightClawClosed();
                    subsystem.leftClawClosed();
                    //left claw close
                })
                .waitSeconds(1)
                .addTemporalMarker( () -> {
                    subsystem.armUp();
                    //left claw close
                })
                .waitSeconds(0.5)
                .forward(14)
                .build();

        rightRedFrontSeq = drive.trajectorySequenceBuilder(initRedFrontSeq.end())
                .lineToSplineHeading(new Pose2d(22,-48, Math.toRadians(90))) //spline to according side
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
                .back(10)
                .waitSeconds(2)
                .addTemporalMarker( () -> {
                    subsystem.armUp();
                })
                .lineToSplineHeading(new Pose2d(51,-43, Math.toRadians(0)))
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
                .strafeRight(16)
                .turn(Math.toRadians(90))
                .addTemporalMarker( () -> {
                    subsystem.armDown();
                })
                .back(2)
                .build();

        //NOT DONE
        leftRedFrontSeq = drive.trajectorySequenceBuilder(initRedFrontSeq.end())
                .forward(3)
                .lineToSplineHeading(new Pose2d(10,-34, Math.toRadians(180)))
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
                .back(10)
                .waitSeconds(2)
                .addTemporalMarker( () -> {
                    subsystem.armUp();
                })
                //.strafeRight(20)
                .lineToSplineHeading(new Pose2d(49,-29, Math.toRadians(0))) //adjust depending on location
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
                .strafeRight(6)
                .turn(Math.toRadians(90))
                .addTemporalMarker( () -> {
                    subsystem.armDown();
                })
                .back(2)
                .build();

        centreRedFrontSeq = drive.trajectorySequenceBuilder(initRedFrontSeq.end())
                .lineToSplineHeading(new Pose2d(35,-24, Math.toRadians(180)))
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
                .back(10)
                .waitSeconds(2)
                .addTemporalMarker( () -> {
                    subsystem.armUp();
                    subsystem.rightClawClosed();
                })
                //.strafeRight(20)
                .lineToSplineHeading(new Pose2d(50,-37, Math.toRadians(0))) //adjust depending on location
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
                    subsystem.rightClawClosed();
                })
                .waitSeconds(2)
                .strafeRight(10)
                .turn(Math.toRadians(90))
                .addTemporalMarker( () -> {
                    subsystem.armDown();
                })
                .back(2)
                .build();

    }

    public void redBack(SampleMecanumDrive drive, Subsystem subsystem) {
        drive.setPoseEstimate(redBackPose);

        initRedBackSeq = drive.trajectorySequenceBuilder(redBackPose)
                .addTemporalMarker( () -> {
                    subsystem.rightClawClosed();
                    subsystem.leftClawClosed();
                    //left claw close
                })
                .waitSeconds(1)
                .addTemporalMarker( () -> {
                    subsystem.armUp();
                    //left claw close
                })
                .waitSeconds(0.5)
                .forward(14)
                .build();

        rightRedBackSeq = drive.trajectorySequenceBuilder(redBackPose)
                .lineToSplineHeading(new Pose2d(-34,-34, Math.toRadians(0)))
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
                .strafeLeft(25)
                .forward(80)
                .lineToSplineHeading(new Pose2d(51,-43, Math.toRadians(0)))
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
                .strafeRight(16)
                .turn(Math.toRadians(90))
                .addTemporalMarker( () -> {
                    subsystem.armDown();
                })
                .back(2)
                .build();

        centreRedBackSeq = drive.trajectorySequenceBuilder(redBackPose)
                .lineToSplineHeading(new Pose2d(-59,-21, Math.toRadians(0)))
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
                .strafeRight(10)
                .turn(Math.toRadians(90))
                .addTemporalMarker( () -> {
                    subsystem.armDown();
                })
                .back(2)
                .build();

       leftRedBackSeq = drive.trajectorySequenceBuilder(redBackPose)
               .lineToSplineHeading(new Pose2d(-48,-48, Math.toRadians(90))) //spline to according side
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
               .forward(40)
               .strafeRight(80)
               .lineToSplineHeading(new Pose2d(49,-29, Math.toRadians(0))) //adjust depending on location
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
               .strafeRight(6)
               .turn(Math.toRadians(90))
               .addTemporalMarker( () -> {
                   subsystem.armDown();
               })
               .back(2)
               .build();
    }


    public void blueFront(SampleMecanumDrive drive, Subsystem subsystem) {
        drive.setPoseEstimate(blueFrontPose);

        initBlueFrontSeq = drive.trajectorySequenceBuilder(blueFrontPose)
                .addTemporalMarker( () -> {
                    subsystem.rightClawClosed();
                    subsystem.leftClawClosed();
                })
                .waitSeconds(1)
                .addTemporalMarker( () -> {
                    subsystem.armUp();
                    //left claw close
                })
                .waitSeconds(0.5)
                .forward(14)
                .build();

        rightBlueFrontSeq = drive.trajectorySequenceBuilder(initBlueFrontSeq.end())
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


        centreBlueFrontSeq = drive.trajectorySequenceBuilder(initBlueFrontSeq.end())
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

        leftBlueFrontSeq = drive.trajectorySequenceBuilder(initBlueFrontSeq.end())
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
    }

    public void blueBack(SampleMecanumDrive drive, Subsystem subsystem) {
        drive.setPoseEstimate(blueBackPose);


    }
}


