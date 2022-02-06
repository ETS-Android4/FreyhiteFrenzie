package org.firstinspires.ftc.teamcode.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot;

@Autonomous(name = "redBottom", group = "d")
public class redBottom extends LinearOpMode {


    @Override
    public void runOpMode(){
    robot bot = new robot(hardwareMap, this);

        waitForStart();

        // position the robot next to the carousel
        bot.moveStraight(2, 1, 0.3);
        bot.strafe(14, -1, 0.25);

        // spin the carousel
        bot.spinCarousel(-bot.DIRECTION);
        sleep(5000);
        bot.spinCarousel(0);

        // park the robot
        bot.resetEncoders();
        bot.moveStraight(17, 0.4, 1);
        bot.strafe(8,-1,0.3);

    }
}
