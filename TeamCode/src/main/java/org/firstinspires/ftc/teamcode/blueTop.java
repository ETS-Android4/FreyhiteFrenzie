package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "blueTop", group = "f")
public class blueTop extends LinearOpMode {

    robot bot = new robot();

    @Override
    public void runOpMode(){

        waitForStart();
        bot.init(hardwareMap, this);

        // positions the bot next to the carousel
        bot.moveStraight(5, 0.4, 1);
        sleep(1000);
        bot.turnTo(Math.PI/2, 0.45);
        bot.moveStraight(50, 0.4, -1);

        // spins the carousel motor
        bot.spinCarousel(bot.direction);
        sleep(6000);
        bot.spin.setPower(0);

        // parks the robot
        bot.strafe(12, 1, 0.6);
        bot.moveStraight(6, 0.3, -1);

    }
}
