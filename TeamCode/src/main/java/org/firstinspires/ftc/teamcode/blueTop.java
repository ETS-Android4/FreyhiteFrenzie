package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "blueTop", group = "f")
public class blueTop extends LinearOpMode {
    robot bot = new robot();
    @Override
    public void runOpMode(){
        bot.init(hardwareMap, this);
        bot.moveStraight(5, 0.8, 1);
        bot.turnTo(0, 0.8);
        bot.moveStraight(144, 0.8, -1);
        bot.spin.setPower(0.8);
        sleep(6969);
        bot.spin.setPower(0);
        bot.strafe(48, 1, 0.8);
        bot.moveStraight(4, 0.8, -1);

    }
}
