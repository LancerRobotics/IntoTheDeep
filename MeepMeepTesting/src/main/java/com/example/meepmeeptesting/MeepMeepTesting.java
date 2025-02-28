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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(25, -61.5, Math.toRadians(90)))
                .strafeTo(new Vector2d(33, -61.5))
                .strafeTo(new Vector2d(33,0))
                .strafeTo(new Vector2d(41, 0))
                .strafeTo(new Vector2d(41, -55))

                .strafeTo(new Vector2d(41,0))
                .strafeTo(new Vector2d(53,0))
                .strafeTo(new Vector2d(53,-55))

                .strafeTo(new Vector2d(53,0))
                .strafeTo(new Vector2d(61,0))
                .strafeTo(new Vector2d(61,-55))
                .build()

        );

        myBot.setDimensions(16.5,16.5);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}