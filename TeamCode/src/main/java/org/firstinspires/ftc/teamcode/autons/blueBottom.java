package org.firstinspires.ftc.teamcode.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot;

@Autonomous(name = "blueBottom", group = "a")
public class blueBottom extends LinearOpMode {


    @Override
    public void runOpMode(){
    robot bot = new robot(hardwareMap, this);
//        bot.initOpenCV();
        waitForStart();
        bot.resetEncoders();
        // positions the bot next to the carousel
        bot.moveStraight(2, 0.4, 1);
        sleep(1000);
        bot.turnTo(Math.PI/2, 0.35);
        bot.moveStraight(15,0.25,-1);
        // spins the carousel motor
        bot.spinCarousel(bot.DIRECTION);
        sleep(5000);
        bot.spinCarousel(0);

        // parks the robot
        bot.resetEncoders();
        bot.strafe(10, 1, 0.3);
        bot.moveStraight(4,.3,-1);


    }

}
