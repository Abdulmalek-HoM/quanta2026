package com.example.meepmeeptesting2;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-50, -50, Math.toRadians(55)))
                .strafeToLinearHeading(new Vector2d(-37, -13), Math.toRadians(160))
                        .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-30, -31), Math.toRadians(230))
                .waitSeconds(1)

                .strafeToLinearHeading(new Vector2d(-12, -30), Math.toRadians(270))
                .waitSeconds(1)

                .strafeToLinearHeading(new Vector2d(-11,-55), Math.toRadians(270))
                .waitSeconds(1)

                .strafeToLinearHeading(new Vector2d(-12,-30), Math.toRadians(270))
                .waitSeconds(1)


                .strafeToLinearHeading(new Vector2d(-30, -31), Math.toRadians(230))
                .waitSeconds(1)

                .strafeToLinearHeading(new Vector2d(-5, -53), Math.toRadians(90))


                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}