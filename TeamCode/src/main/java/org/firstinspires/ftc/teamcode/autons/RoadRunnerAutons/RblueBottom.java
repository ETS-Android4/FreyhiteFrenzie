package org.firstinspires.ftc.teamcode.autons.RoadRunnerAutons;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.driveConstants;
import org.firstinspires.ftc.teamcode.robot;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "RblueBottom", group = "cccc")

public class RblueBottom extends LinearOpMode{
    OpenCvWebcam webcam;
    @Override
    public void runOpMode() throws InterruptedException{
        robot bot = new robot(hardwareMap, this);
        bot.initOpenCV(webcam);
        telemetry.addData("Barcode: ", bot.barcode);
        telemetry.update();
        Pose2d start = new Pose2d(-27.5,65.45, Math.toRadians(180)); //start position & orientation
        Pose2d endHub;
        bot.setPoseEstimate(start); //set start position
        waitForStart();
        int x = 1;
        if (bot.barcode == 0){
            Trajectory traj0 = bot.trajectoryBuilder(start, true) //build trajectory 0
                    .splineTo(new Vector2d(-4, 36), Math.toRadians(270),
                            bot.getVelocityConstraint(38, driveConstants.MAX_ANG_VEL, driveConstants.TRACK_WIDTH),
                            bot.getAccelerationConstraint(driveConstants.MAX_ACCEL))
                    .build();
            bot.followTrajectory(traj0);
            bot.armToTop();
            sleep(1000);
            bot.wristDrop();
            sleep(1000);
            bot.armReset();
            endHub = traj0.end();
        }
        else if (bot.barcode == 1) {
            Trajectory traj1 = bot.trajectoryBuilder(start, true) //build trajectory 1
                    .splineTo(new Vector2d(-4, 44.5), Math.toRadians(270),
                            bot.getVelocityConstraint(38, driveConstants.MAX_ANG_VEL, driveConstants.TRACK_WIDTH),
                            bot.getAccelerationConstraint(driveConstants.MAX_ACCEL))
                    .build();
            bot.followTrajectory(traj1);
            bot.armToMiddle();
            sleep(1000);
            bot.wristDrop();
            sleep(1000);
            bot.wristReset();
            bot.armReset();
            endHub = traj1.end();
        }
        else {
            Trajectory traj2 = bot.trajectoryBuilder(start, true) //build trajectory 2
                    .splineTo(new Vector2d(-4, 45.5),  Math.toRadians(270),
                            bot.getVelocityConstraint(38, driveConstants.MAX_ANG_VEL, driveConstants.TRACK_WIDTH),
                            bot.getAccelerationConstraint(driveConstants.MAX_ACCEL))
                    .build();
            bot.followTrajectory(traj2);
            bot.armToBottom();
            sleep(1000);
            bot.wristDrop();
            sleep(1000);
            bot.armReset();
            endHub = traj2.end();
        }
        sleep(1000);
        Trajectory traj = bot.trajectoryBuilder(endHub)
                .forward(6)
                .build();
        Trajectory trajBack = bot.trajectoryBuilder(traj.end()) //build first half of trajback with a midpoint
                .splineToLinearHeading(new Pose2d(-44,47), Math.toRadians(0),
                        bot.getVelocityConstraint(30, driveConstants.MAX_ANG_VEL, driveConstants.TRACK_WIDTH),
                        bot.getAccelerationConstraint(driveConstants.MAX_ACCEL))
                .build();
        bot.followTrajectory(trajBack);

        Trajectory last = bot.trajectoryBuilder(trajBack.end())
                .back(10)
                .build();
        bot.followTrajectory(last);

        bot.spinCarousel(bot.DIRECTION);
        sleep(5000);
        bot.spinCarousel(0);

        Trajectory trajPark = bot.trajectoryBuilder(trajBack.end())
            .strafeRight(23).build();
        Trajectory trajPark1 = bot.trajectoryBuilder(trajPark.end())
                .back(9).build();
        bot.followTrajectory(trajPark);
        bot.followTrajectory(trajPark1);

    }
}
