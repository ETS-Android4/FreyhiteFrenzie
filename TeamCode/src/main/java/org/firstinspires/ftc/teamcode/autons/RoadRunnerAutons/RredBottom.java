package org.firstinspires.ftc.teamcode.autons.RoadRunnerAutons;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.driveConstants;
import org.firstinspires.ftc.teamcode.robot;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "RredBottom", group = "cccc")

public class RredBottom extends LinearOpMode {
    OpenCvWebcam webcam;
    @Override
    public void runOpMode() throws InterruptedException{
        robot bot = new robot(hardwareMap, this);
        bot.initOpenCV(webcam);
        waitForStart();
        Pose2d start = new Pose2d(-27.5,-65.45, Math.toRadians(180)); //start position & orientation
        Pose2d endHub;
        bot.setPoseEstimate(start); //set start position
        int x = 1;
        if (bot.barcode == 0){
            Trajectory traj0 = bot.trajectoryBuilder(start, true) //build trajectory 0
                    .splineTo(new Vector2d(-6, -37), Math.toRadians(90),
                            bot.getVelocityConstraint(37, driveConstants.MAX_ANG_VEL, driveConstants.TRACK_WIDTH),
                            bot.getAccelerationConstraint(driveConstants.MAX_ACCEL))
                    .build();
            bot.followTrajectory(traj0);
            endHub = traj0.end();
            bot.armToTop();
            sleep(1000);
            bot.wristDrop();
            sleep(1000);
            bot.armReset();
            sleep(1000);
        }
        else if (bot.barcode == 1) {
            Trajectory traj1 = bot.trajectoryBuilder(start,true) //build trajectory 1
                    .splineTo(new Vector2d(-6, -42), Math.toRadians(90),
                            bot.getVelocityConstraint(37, driveConstants.MAX_ANG_VEL, driveConstants.TRACK_WIDTH),
                            bot.getAccelerationConstraint(driveConstants.MAX_ACCEL))
                    .build();
            bot.followTrajectory(traj1);
            endHub = traj1.end();
            bot.armToMiddle();
            sleep(1000);
            bot.wristDrop();
            sleep(1000);
            bot.armReset();
            sleep(1000);
        }
        else {
            Trajectory traj2 = bot.trajectoryBuilder(start, true) //build trajectory 2
                    .splineTo(new Vector2d(-6, -45),  Math.toRadians(90),
                            bot.getVelocityConstraint(37, driveConstants.MAX_ANG_VEL, driveConstants.TRACK_WIDTH),
                            bot.getAccelerationConstraint(driveConstants.MAX_ACCEL))
                    .build();
            bot.followTrajectory(traj2);
            endHub = traj2.end();
            bot.armToBottom();
            sleep(1000);
            bot.wristDrop();
            sleep(1000);
            bot.armReset();
            sleep(1000);
        }

        Trajectory trajBack = bot.trajectoryBuilder(endHub) //build trajectory 0   //the top right corner of bottom left square (where carosuel is positioned)
                .splineTo(new Vector2d(-53, -53),  Math.toRadians(90),
                        bot.getVelocityConstraint(30, driveConstants.MAX_ANG_VEL, driveConstants.TRACK_WIDTH),
                        bot.getAccelerationConstraint(driveConstants.MAX_ACCEL))
                .build();
        bot.followTrajectory(trajBack);

        Trajectory last = bot.trajectoryBuilder(trajBack.end())
                .back(12, bot.getVelocityConstraint(30, driveConstants.MAX_ANG_VEL, driveConstants.TRACK_WIDTH),
                        bot.getAccelerationConstraint(driveConstants.MAX_ACCEL))
                .build();
        bot.followTrajectory(last);

        bot.spinCarousel(-bot.DIRECTION);
        sleep(5000);
        bot.spinCarousel(0);
        //bot.followTrajectory(your mom)
        Trajectory trajPark = bot.trajectoryBuilder(trajBack.end())
                .forward(10, bot.getVelocityConstraint(30, driveConstants.MAX_ANG_VEL, driveConstants.TRACK_WIDTH),
                        bot.getAccelerationConstraint(driveConstants.MAX_ACCEL)).build();
        Trajectory trajPark1 = bot.trajectoryBuilder(trajPark.end())
                .strafeLeft(8).build();
        bot.followTrajectory(trajPark);
        bot.followTrajectory(trajPark1);
    }
}

