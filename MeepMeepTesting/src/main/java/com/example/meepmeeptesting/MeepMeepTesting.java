package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity SpecimenBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedDark())
                .build();
        RoadRunnerBotEntity SampleBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedLight())
                .build();

        /*
        SpecimenBot.runAction(SpecimenBot.getDrive().actionBuilder(new Pose2d(9, -61.5, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(0,-35), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(10, -40, Math.toRadians(90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(35, -30, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(35, -20, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(45, -10, Math.toRadians(90)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(45, -60, Math.toRadians(90)), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(45,-50), Math.toRadians(-90))
                .strafeToSplineHeading(new Vector2d(0,-35), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(10, -40, Math.toRadians(90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(35, -30, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(35, -20, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(53, -10, Math.toRadians(90)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(53, -60, Math.toRadians(90)), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(53,-50), Math.toRadians(-90))
                .strafeToSplineHeading(new Vector2d(0,-35), Math.toRadians(90))
                .build());
         */
        double spec1 = 46;
        double spec2 = 53;

        SpecimenBot.runAction(SpecimenBot.getDrive().actionBuilder(new Pose2d(spec1, -55, Math.toRadians(90)))
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(spec1-10, -45, Math.toRadians(0)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(spec1-5, -35, Math.toRadians(-90)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(spec1, -45, Math.toRadians(-90)), Math.toRadians(-90))
                .strafeToSplineHeading(new Vector2d(spec1, -55), Math.toRadians(-90))
                .build()
        );

        SampleBot.runAction(SampleBot.getDrive().actionBuilder(new Pose2d(-32, -61.5, Math.toRadians(180)))
                        .waitSeconds(2.1)
                .strafeToLinearHeading(new Vector2d(-56, -57), Math.toRadians(225))
                        .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(-59,-41), Math.toRadians(88))
                        .waitSeconds(1.75)
                .strafeToLinearHeading(new Vector2d(-56, -57), Math.toRadians(230))
                        .waitSeconds(0.4)
                .strafeToLinearHeading(new Vector2d(-47.5, -41.5), Math.toRadians(86))
                        .waitSeconds(1.75)
                .strafeToLinearHeading(new Vector2d(-56, -57), Math.toRadians(230))
                        .waitSeconds(0.4)
                .strafeToSplineHeading(new Vector2d(-50,-20), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-20,0), Math.toRadians(0))
                .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(SpecimenBot)
                .addEntity(SampleBot)
                .start();
    }
}