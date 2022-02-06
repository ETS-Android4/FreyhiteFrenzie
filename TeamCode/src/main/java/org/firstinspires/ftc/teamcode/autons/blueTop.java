package org.firstinspires.ftc.teamcode.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot;

@Autonomous(name = "blueTop", group = "f")
public class blueTop extends LinearOpMode {


    @Override
    public void runOpMode(){
    robot bot = new robot(hardwareMap, this);

        waitForStart();

        // positions the bot next to the carousel
        bot.moveStraight(5, 0.4, 1);
        sleep(1000);
        bot.turnTo(Math.PI/2, 0.45);
        bot.moveStraight(50, 0.4, -1);

        // spins the carousel motor
        bot.spinCarousel(bot.DIRECTION);
        sleep(6000);
        bot.spinCarousel(0);

        // parks the robot
        bot.strafe(12, 1, 0.6);
        bot.moveStraight(6, 0.3, -1);

    }
}
