package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        /*
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(9, -61.5, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(0,-35), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(10, -40, Math.toRadians(90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(35, -30, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(35, -20, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(45, -10, Math.toRadians(90)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(45, -50, Math.toRadians(90)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(45, -10, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(55, -10, Math.toRadians(90)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(55, -50, Math.toRadians(90)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(55, -10, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(62, -10, Math.toRadians(90)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(62, -50, Math.toRadians(90)), Math.toRadians(-90))
                .build());
         */

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(9, -61.5, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(0,-35), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(10, -40, Math.toRadians(90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(35, -30, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(35, -20, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(45, -10, Math.toRadians(90)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(45, -50, Math.toRadians(90)), Math.toRadians(-90))
                .strafeToSplineHeading(new Vector2d(45,-55), Math.toRadians(-90))
                .strafeToSplineHeading(new Vector2d(0,-35), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(10, -40, Math.toRadians(90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(35, -30, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(35, -20, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(53, -10, Math.toRadians(90)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(53, -50, Math.toRadians(90)), Math.toRadians(-90))
                .strafeToSplineHeading(new Vector2d(53,-55), Math.toRadians(-90))
                .strafeToSplineHeading(new Vector2d(0,-35), Math.toRadians(90))

                .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}