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
                                .lineToSplineHeading(new Pose2d(12,20, Math.toRadians(0)))
                                .strafeLeft(4)//spline to according side
                                .addTemporalMarker( () -> {
                                    //subsystem.armDown();
                                    //right claw open
                                })
                                .waitSeconds(0.5)
                                .addTemporalMarker( () -> {
                                    //subsystem.armUp();
                                    //right claw close
                                })
                                .lineToSplineHeading(new Pose2d(48,35, Math.toRadians(0))) //adjust depending on location
                                .addTemporalMarker( () -> {
                                    //subsystem.slidePositionTo(100);
                                    //left claw open
                                })
                                .waitSeconds(0.5)
                                .addTemporalMarker( () -> {
                                    //subsystem.slideDown();
                                    //left claw close
                                })
                                .strafeLeft(22)
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