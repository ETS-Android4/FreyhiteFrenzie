package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "redBottom", group = "d")
public class redBottom extends LinearOpMode {
    robot bot = new robot();
    @Override
    public void runOpMode(){
        bot.init(hardwareMap, this);
        bot.moveStraight(5, 0.8, 1);
        bot.strafe(48, -1, 0.8);
        bot.spin.setPower(0.8);
        sleep(6969);
        bot.spin.setPower(0);
        bot.moveStraight(10, 0.8, 1);
        bot.strafe(4, -1, 0.8);

    }
}
