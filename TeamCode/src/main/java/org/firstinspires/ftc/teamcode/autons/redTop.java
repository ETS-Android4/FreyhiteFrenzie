package org.firstinspires.ftc.teamcode.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot;

@Autonomous(name = "redTop", group = "g")
public class redTop extends LinearOpMode {


    @Override
    public void runOpMode(){
    robot bot = new robot(hardwareMap, this);

        waitForStart();

        // position the robot next to the carousel
        bot.strafe(39, -1, 0.45);

        // spin the carousel
        bot.spinCarousel(-bot.DIRECTION);
        sleep(5000);
        bot.spinCarousel(0);

        // park the robot
        bot.turnTo(0, 0.6);
        bot.resetEncoders();
        bot.moveStraight(14, 0.45, 1);
        bot.strafe(5,-1,0.45);
    }
}
