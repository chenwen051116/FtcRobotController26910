package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

// This one is for the yellow block side
public class MeepMeepTesting2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        int dl=1;
        int ds=1;
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(360), Math.toRadians(360), 11.692)
                .setDimensions(15.259,20.632)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(34.9, 56.9, Math.toRadians(270)))
                        .lineToLinearHeading(new Pose2d(26, 15, Math.toRadians(180)) )
                        .waitSeconds(dl)
                        .waitSeconds(ds)
                        .lineToLinearHeading(new Pose2d(36, 40.8, Math.toRadians(223)) )
                        .lineToLinearHeading(new Pose2d(55.7, 55.7, Math.toRadians(223)) )
                        .waitSeconds(ds)
                        .lineToLinearHeading(new Pose2d(55, 42, Math.toRadians(250)) )
                        .waitSeconds(dl)
                        .lineToLinearHeading(new Pose2d(55.7, 55.7, Math.toRadians(223)) )
                        .waitSeconds(ds)
                        .lineToLinearHeading(new Pose2d(44.7, 41.7, Math.toRadians(310)) )
                        .waitSeconds(dl)
                        .lineToLinearHeading(new Pose2d(55.7, 55.7, Math.toRadians(223)) )
                        .waitSeconds(ds)
                        .lineToLinearHeading(new Pose2d(50.7, 30.7, Math.toRadians(350)) )
                        .waitSeconds(dl)
                        .lineToLinearHeading(new Pose2d(55.7, 55.7, Math.toRadians(223)) )
                        .waitSeconds(ds)
                        .lineToLinearHeading(new Pose2d(26, 10, Math.toRadians(180)) )
                        .waitSeconds(dl)
                        .lineToLinearHeading(new Pose2d(55.7, 55.7, Math.toRadians(223)) )
                        .waitSeconds(ds)
                        .lineToLinearHeading(new Pose2d(26, 15, Math.toRadians(180)) )
                        .waitSeconds(dl)
                        .lineToLinearHeading(new Pose2d(55.7, 55.7, Math.toRadians(223)) )
                        .waitSeconds(ds)
                        .build())
                ;



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
