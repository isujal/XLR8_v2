package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Arrays;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12)
                .build();
        VelConstraint speedDown = new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(90)));

//TODO : NEAR AUTO
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-36, -55, -90))
                //TODO : Preload Shoot
                .strafeToLinearHeading(new Vector2d(-7, -16), Math.toRadians(-90))
//                .strafeToConstantHeading(new Vector2d(-14, 16))
                //TODO : Pick First Motif from loading zone
                .strafeToLinearHeading(new Vector2d(18, -32), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(18, -60), Math.toRadians(-90))

//                .strafeToLinearHeading(new Vector2d(-7, 16), Math.toRadians(90))


                .waitSeconds(0.3)

                // adity don ritesh mithu

                        .setReversed(false)
                .strafeToLinearHeading(new Vector2d(-7, -16), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(18,-60,Math.toRadians(-130)), Math.toRadians(-145))
                .strafeToLinearHeading(new Vector2d(23, -61), Math.toRadians(-145))

//                .strafeToLinearHeading(new Vector2d(-7, 16), Math.toRadians(90))


                .waitSeconds(0.3)

                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(43, 12), Math.toRadians(90))







//                .strafeToConstantHeading(new Vector2d(-14, 16))
                //TODO : Pick First Motif from loading zone

//                .splineToConstantHeading(new Vector2d(6,56),Math.toRadians(145))
//
//                .strafeToLinearHeading(new Vector2d(-11.5, 16), Math.toRadians(90))
////                .strafeToConstantHeading(new Vector2d(-14, 16))
//                //TODO : Pick First Motif from loading zone
//
//                .splineToConstantHeading(new Vector2d(6,56),Math.toRadians(155))
//                .waitSeconds(0.25)
//                .splineToConstantHeading(new Vector2d(-11.5, 42.5), Math.toRadians(70))
//                .strafeToLinearHeading(new Vector2d(-11.5, 42.5), Math.toRadians(90))
//                .strafeToLinearHeading(new Vector2d(2, 54), Math.toRadians(90))
//
//
//                //TODO : First Motif Shoot
//                .strafeToLinearHeading(new Vector2d(-20, 32), Math.toRadians(90))
//                .waitSeconds(0.25)
//                //TODO : Pick from 2nd SpikeMark and Shoot
//                .strafeToLinearHeading(new Vector2d(11.5, 32), Math.toRadians(90))
//                .waitSeconds(0.25)
//                .strafeToLinearHeading(new Vector2d(11.5, 42.5), Math.toRadians(90))
//                //TODO : Second Motif Shoot
//                .strafeToLinearHeading(new Vector2d(-20, 32), Math.toRadians(90))
//                .waitSeconds(0.25)

                .build());

//TODO : FAR AUTO
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, 15, Math.toRadians(90)))
//                //TODO : Preload Shoot and Pick from Third Spike Mark
//                .strafeToLinearHeading(new Vector2d(47, 17), Math.toRadians(90))
//
//                .strafeToLinearHeading(new Vector2d(6, 30), Math.toRadians(90))
//                .strafeToConstantHeading(new Vector2d(6,60))
//                .strafeToConstantHeading(new Vector2d(6,30))
////                .splineToConstantHeading(new Vector2d(70, 73))  //4.34S
//
//                //TODO : Shoot Second Motif
//                .strafeToLinearHeading(new Vector2d(47, 17), Math.toRadians(90))
////                .waitSeconds(0.5)
////                .waitSeconds(0.5)
//                .strafeToLinearHeading(new Vector2d(30, 30), Math.toRadians(90))
//                                .strafeToConstantHeading(new Vector2d(30,60))
////                .splineToConstantHeading(new Vector2d(70, 73))  //4.34S
//
//                //TODO : Shoot Second Motif
//                .strafeToLinearHeading(new Vector2d(47, 17), Math.toRadians(90))
//
//
////                .strafeToLinearHeading(new Vector2d(47, -47), Math.toRadians(-45))  //4.34S
//////                .waitSeconds(0.5)
//////                .waitSeconds(0.5)
//////                //TODO : Pick from Second Spike Mark
////                .strafeToConstantHeading(new Vector2d(8, -60))
//////                .waitSeconds(0.5)
//////                .waitSeconds(0.5)
////                .strafeToConstantHeading(new Vector2d(47, -9))
//
////                .splineToConstantHeading(new Vector2d(12, 42), Math.toRadians(100))
//                .waitSeconds(1)
//                //TODO : Gate Open
////                .splineToConstantHeading(new Vector2d(7, 55), Math.toRadians(180))
//                //TODO : Shoot Third Motif
////                .strafeToConstantHeading(new Vector2d(-5, 12))  //4.34S
////                .strafeToLinearHeading(new Vector2d(50, 13), Math.toRadians(90))  //4.34S
//
//                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(1.99f)
                .addEntity(myBot)
                .start();
    }
}
