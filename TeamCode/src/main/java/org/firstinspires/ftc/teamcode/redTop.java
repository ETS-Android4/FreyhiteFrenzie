package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "redTop", group = "g")
public class redTop extends LinearOpMode {

    robot bot = new robot();

    @Override
    public void runOpMode(){

        waitForStart();
        bot.init(hardwareMap, this);

        // position the robot next to the carousel
        bot.strafe(39, -1, 0.45);

        // spin the carousel
        bot.spinCarousel(-bot.direction);
        sleep(5000);
        bot.spin.setPower(0);

        // park the robot
        bot.turnTo(0, 0.6);
        bot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.moveStraight(14, 0.45, 1);
        bot.strafe(5,-1,0.45);
    }
}
