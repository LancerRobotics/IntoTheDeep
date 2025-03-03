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

        // initialHookPos is (0,-34);
        Vector2d getPos = new Vector2d(54,-48.5);
        double hookAngle = Math.toRadians(90), grabAngle = Math.toRadians(270);
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-39.5, -61.5, Math.toRadians(180)))
                .strafeTo(new Vector2d(-39.5, -55))
                .strafeTo(new Vector2d(-59, -59))
                .turnTo(Math.toRadians(225))

                .waitSeconds(1)
                //.strafeTo(new Vector2d(-10, -34))
                .waitSeconds(0.6)
                .waitSeconds(1)
                .waitSeconds(1.5)

                .waitSeconds(1)
                .waitSeconds(0.5)
                .waitSeconds(0.5)
                // PARKING TIME!!!!!!!!!!!!!
                .strafeTo(new Vector2d(-59,-56))
                .turnTo(Math.toRadians(0))
                .strafeTo(new Vector2d(55,-56))
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