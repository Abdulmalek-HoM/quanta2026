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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(58, 24, Math.toRadians(180)))

                //1. Shooting position
                .strafeToLinearHeading(new Vector2d(-57, 12), Math.toRadians(90))


                .waitSeconds(1)

                //2. Balls smart intake take over position (start from y=30 up to y=57 and head back )
                .splineTo(new Vector2d(12, 30), Math.toRadians(90))
                .waitSeconds(1)

                //3. Return to Shooting position
                .splineTo(new Vector2d(-57, 12), Math.toRadians(90))
                .waitSeconds(1)

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}