package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        Pose2d startPose = //new Pose2d(12, 60, Math.toRadians(270));
                new Pose2d(-36, -60, Math.toRadians(90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .addTemporalMarker( () -> {
                                    //subsystem.rightClawClosed();
                                    //subsystem.leftClawClosed();
                                    //left claw close
                                })
                                .waitSeconds(1)
                                .addTemporalMarker( () -> {
                                    //subsystem.armUp();
                                    //left claw close
                                })
                                .waitSeconds(0.5)
                                .forward(14)

                                // center

                                .lineToSplineHeading(new Pose2d(-48,-48, Math.toRadians(90))) //spline to according side
                                .addTemporalMarker( () -> {
                                    //subsystem.armDown();
                                })
                                .back(3)
                                .waitSeconds(1.5)
                                .forward(12)
                                .addTemporalMarker( () -> {
                                    //subsystem.leftClawOpen();
                                })
                                .waitSeconds(1.5)
                                .back(10)
                                .waitSeconds(2)
                                .addTemporalMarker( () -> {
                                    //subsystem.armUp();
                                })
                                .strafeLeft(12)
                                .forward(40)
                                .strafeRight(80)
                                .turn(Math.toRadians(180))
                                .strafeLeft(40)
                                .addTemporalMarker( () -> {
                                    //subsystem.rightClawOpen();
                                })
                                .build()



                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}