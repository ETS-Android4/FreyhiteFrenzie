package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "redTop", group = "g")
public class redTop extends LinearOpMode {
    robot bot = new robot();
    @Override
    public void runOpMode(){
        bot.init(hardwareMap, this);
        bot.moveStraight(5, 0.8, 1);
        bot.turnTo(Math.PI/2, 0.8);
        bot.strafe(144, -1, 0.8);
        bot.spin.setPower(0.8);
        sleep(6969);
        bot.spin.setPower(0);
        bot.moveStraight(10, 0.8, 1);
        bot.strafe(4, -1, 0.8);

    }
}
