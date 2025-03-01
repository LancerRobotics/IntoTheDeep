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
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, -61.75, hookAngle))
                // Move to hook
                .strafeTo(new Vector2d(-10, -34))
                // hook

                // Move to human player
                .turnTo(grabAngle)
                .strafeTo(getPos)
                // grab block

                // Move to hook
                .strafeTo(new Vector2d(-5, -34))
                .turnTo(hookAngle)
                // hook

                // Move to human player
                .turnTo(grabAngle)
                .strafeTo(getPos)
                //grab block

                // Move to hook
                .strafeTo(new Vector2d(0, -34))
                .turnTo(hookAngle)
                // hook

                // Move to human player
                .turnTo(grabAngle)
                .strafeTo(getPos)
                // grab block

                // Move to hook
                .turnTo(hookAngle)
                .strafeTo(new Vector2d(5, -34))
                //hook

                // PARKING TIME!!!!!!!!!!!!!
                .strafeTo(new Vector2d(60,-56))
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