package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-39, -60.5, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-58, -60, Math.toRadians(0)))
                                //.splineToSplineHeading(new Pose2d(-58, -24, Math.toRadians(180)), Math.toRadians(0))
                                .lineTo(new Vector2d(-58,-24))
                                .forward(3)
                                // Drop Cone
                                .back(2)
                                // Turn claw the other way
                                .lineTo(new Vector2d(-57, -12))
                                .back(2)
                                // Grab cone
                                // turn claw with cone to drop
                                .lineToLinearHeading(new Pose2d(-24, -12, Math.toRadians(270)))
                                .back(3)
                                // drop cone
                                .forward(3)
                                .lineToLinearHeading(new Pose2d(-57, -12, Math.toRadians(180)))
                                .forward(3)
                                // grab cone
                                .back(3)
                                .lineToLinearHeading(new Pose2d(-24, -12, Math.toRadians(90)))
                                .back(3)
                                // drop cone
                                .forward(3)

                                // Vision 1

                               /* .lineTo(new Vector2d(-59,-12))
                                .lineTo(new Vector2d(-59,-20))*/
                                // Vision 2
/*
                                .lineTo(new Vector2d(-36,-12))
                                .lineTo(new Vector2d(-36,-20))*/

                                // Vision 3
                                .lineTo(new Vector2d(-12,-12))
                                .lineTo(new Vector2d(-12,-20))


                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}