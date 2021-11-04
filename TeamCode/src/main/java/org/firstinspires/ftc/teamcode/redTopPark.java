package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "redTopPark", group = "h")
public class redTopPark extends LinearOpMode {
    robot bot = new robot();
    @Override
    public void runOpMode(){
        bot.init(hardwareMap, this);
        bot.moveStraight(48, 0.8, 1);
        bot.strafe(192, -1, 0.8);
    }
}
