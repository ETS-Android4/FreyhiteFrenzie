package org.firstinspires.ftc.teamcode.autons.RoadRunnerTesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.driveConstants;
import org.firstinspires.ftc.teamcode.robot;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name = "SplineTest", group = "S")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        robot wucru = new robot(hardwareMap, this);
/*
Dear Heinz Tomato Ketchup,

    light mode is superior. dark mode sucks

   Love,
   Wucru
   (mastercoder's master,
   daddy wu
   father wu
   ryan my boy
   master wu
   wuwu
   uwu
   woodle poodle
   uwu_cru_uwu)
 */
        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj = wucru.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(48, 24), 0,
                        wucru.getVelocityConstraint(37, driveConstants.MAX_ANG_VEL, driveConstants.TRACK_WIDTH),
                        wucru.getAccelerationConstraint(driveConstants.MAX_ACCEL))
                .build();

        wucru.followTrajectory(traj);

        sleep(2000);

        wucru.followTrajectory(
                wucru.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180), wucru.getVelocityConstraint(37, driveConstants.MAX_ANG_VEL, driveConstants.TRACK_WIDTH),
                                wucru.getAccelerationConstraint(driveConstants.MAX_ACCEL))
                        .build()
        );
    }
}