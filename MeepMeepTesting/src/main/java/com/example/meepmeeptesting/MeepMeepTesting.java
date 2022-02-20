package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-27.5,65.45, Math.toRadians(180)) //start position & orientation
)
                                .splineTo(new Vector2d(-12, 36), Math.toRadians(270))
                                .waitSeconds(2)
                                .splineToLinearHeading(new Pose2d(22, 61), Math.toRadians(0))
                                .forward(30)
                                .waitSeconds(1)
                                .back(30)
                                .splineTo(new Vector2d(-12, 36), Math.toRadians(270))
                                .waitSeconds(1)
                                .splineToLinearHeading(new Pose2d(22, 61), Math.toRadians(0))
                                .forward(30)
                                .waitSeconds(1)
                                .back(30)
                                .splineTo(new Vector2d(-12, 36), Math.toRadians(270))
                                .waitSeconds(1)
                                .splineToLinearHeading(new Pose2d(-59,57), Math.toRadians(0))
                                .waitSeconds(2)
                                .strafeRight(20)
                                //.splineTo(new Vector2d(-51, 54.55), Math.toRadians(180))
                                .build()
                )
                ;

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }

}