package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        //red front right
        /*Pose2d startPose = new Pose2d(12, -60, Math.toRadians(90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .lineToSplineHeading(new Pose2d(32,-38, Math.toRadians(180)))
                                .back(3)
                                .lineToSplineHeading(new Pose2d(51,-43, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(48,-58, Math.toRadians(180)))
                                .build()
                );*/




        //red front left
        /*Pose2d startPose = new Pose2d(12, -60, Math.toRadians(90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .lineToSplineHeading(new Pose2d(12,-34, Math.toRadians(180)))
                                .forward(4)
                                .back(3)
                                .lineToSplineHeading(new Pose2d(49,-29, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(48,-58, Math.toRadians(180)))
                                .build()
                );



        //red front center
        /*Pose2d startPose = new Pose2d(12, -60, Math.toRadians(90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .lineToSplineHeading(new Pose2d(21,-26, Math.toRadians(180)))
                                .back(3)
                                .lineToSplineHeading(new Pose2d(50,-37, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(48,-58, Math.toRadians(180)))
                                .build()
                );
         */

        /*
        blue front left
        Pose2d startPose = new Pose2d(12, 60, Math.toRadians(270));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .lineToSplineHeading(new Pose2d(30,38, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(50,38, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(48,58, Math.toRadians(180)))
                                .build()
                );
        */

        /*
        blue front right
        Pose2d startPose = new Pose2d(12, 60, Math.toRadians(270));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .lineToSplineHeading(new Pose2d(12,34, Math.toRadians(180)))
                                .forward(3)
                                .lineToSplineHeading(new Pose2d(49,28, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(48,58, Math.toRadians(180)))
                                .build()
                );
        */


        //blue front center
        Pose2d startPose = new Pose2d(12, 60, Math.toRadians(270));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .lineToSplineHeading(new Pose2d(17,22, Math.toRadians(180)))
                                .addTemporalMarker(() -> {
                                    //place pixel
                                })
                                .waitSeconds(1)
                                .back(3) //might not be needed
                                .lineToSplineHeading(new Pose2d(48,35, Math.toRadians(180)))
                                .addTemporalMarker(() -> {
                                    //arm into scoring position
                                })
                                .waitSeconds(0.5)
                                .addTemporalMarker(() -> {
                                    //slide up
                                })
                                .waitSeconds(3)
                                .addTemporalMarker(() -> {
                                    //place
                                })
                                .waitSeconds(0.1)
                                .addTemporalMarker(() -> {
                                    //slide down
                                })
                                .lineToSplineHeading(new Pose2d(48,58, Math.toRadians(180)))
                                .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}