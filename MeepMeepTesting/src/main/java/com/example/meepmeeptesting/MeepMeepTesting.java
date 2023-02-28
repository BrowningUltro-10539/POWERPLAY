package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(126), Math.toRadians(126), 9.75)
                .setDimensions(10, 13)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(33, -62, Math.toRadians(270)))
                                .lineTo(new Vector2d(37, -12))
                                .splineTo(new Vector2d(36, -7), Math.toRadians(140))
                                .waitSeconds(1)
                                .splineTo(new Vector2d(48, -12.5), Math.toRadians(0))
                                .lineTo(new Vector2d(62.5, -12.5))
                                .waitSeconds(1)
                                .lineTo(new Vector2d(48, -12.5))
                                .splineTo(new Vector2d(32, -5), Math.toRadians(140))


                                //Driving to pole to drop off preload
//                                .lineToSplineHeading(new Pose2d(36, -20, Math.toRadians(270)))
//                                .splineTo(new Vector2d(31, -5), Math.toRadians(145))
//                                .waitSeconds(0.5)
//
//                                //Driving to cone stack to pick up first cone
//                                .splineTo(new Vector2d(45, -11.5), Math.toRadians(0))
//                                .lineTo(new Vector2d(62, -11.5))
//                                .waitSeconds(0.5)
//
//                                //Driving back to pole to deliver first cone
//                                .lineToSplineHeading(new Pose2d(45, -11.5, Math.toRadians(0)))
//                                .splineTo(new Vector2d(31, -5), Math.toRadians(145))
//                                .waitSeconds(0.5)
//
//                                //Driving back to cone stack to pick up second cone
//                                .splineTo(new Vector2d(45, -11.5), Math.toRadians(0))
//                                .lineTo(new Vector2d(62, -11.5))
//                                .waitSeconds(0.5)
//
//                                //Driving back to pole to deliver second cone
//                                .lineToSplineHeading(new Pose2d(45, -11.5, Math.toRadians(0)))
//                                .splineTo(new Vector2d(31, -5), Math.toRadians(145))
//                                .waitSeconds(0.5)
//
//                                //Driving back to cone stack to pick up third cone
//                                .splineTo(new Vector2d(45, -11.5), Math.toRadians(0))
//                                .lineTo(new Vector2d(62, -11.5))
//                                .waitSeconds(0.5)
//
//                                //Driving back to pole to deliver third cone
//                                .lineToSplineHeading(new Pose2d(45, -11.5, Math.toRadians(0)))
//                                .splineTo(new Vector2d(31, -5), Math.toRadians(145))
//                                .waitSeconds(0.5)
//
//                                //Driving back to cone stack to pick up fourth cone
//                                .splineTo(new Vector2d(45, -11.5), Math.toRadians(0))
//                                .lineTo(new Vector2d(62, -11.5))
//                                .waitSeconds(0.5)
//
//                                //Driving back to pole to deliver fourth cone
//                                .lineToSplineHeading(new Pose2d(45, -11.5, Math.toRadians(0)))
//                                .splineTo(new Vector2d(31, -5), Math.toRadians(145))
//                                .waitSeconds(0.5)
//
//                                //Driving back to cone stack to pick up fifth cone
//                                .splineTo(new Vector2d(45, -11.5), Math.toRadians(0))
//                                .lineTo(new Vector2d(62, -11.5))
//                                .waitSeconds(0.5)
//
//                                //Driving back to pole to deliver fifth cone
//                                .lineToSplineHeading(new Pose2d(45, -11.5, Math.toRadians(0)))
//                                .splineTo(new Vector2d(31, -5), Math.toRadians(145))
//                                .waitSeconds(0.5)

                                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}