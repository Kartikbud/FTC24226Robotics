package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        Pose2d startPose = new Pose2d(12, 60, Math.toRadians(270));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .addTemporalMarker( () -> {
                                    //subsystem.rightClawClosed();
                                    //subsystem.leftClawClosed();
                                })
                                .waitSeconds(1)
                                .addTemporalMarker( () -> {
                                    //subsystem.armUp();
                                })
                                .forward(14)
                                .lineToSplineHeading(new Pose2d(35,21, Math.toRadians(180)))
                                //.strafeRight(8)
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
                                //.strafeRight(20)
                                .lineToSplineHeading(new Pose2d(46,38, Math.toRadians(0))) //adjust depending on location
                                .addTemporalMarker( () -> {
                                    //subsystem.slidePositionTo(1000);
                                })
                                .waitSeconds(5)
                                .addTemporalMarker( () -> {
                                    //subsystem.rightClawOpen();
                                })
                                .waitSeconds(1)
                                .addTemporalMarker( () -> {
                                    //subsystem.slideDown();
                                })
                                .waitSeconds(2)
                                .strafeLeft(18)
                                .turn(Math.toRadians(-90))
                                .build()



                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}