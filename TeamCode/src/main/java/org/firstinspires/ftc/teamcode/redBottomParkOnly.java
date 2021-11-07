package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "redBottomParkOnly", group = "e")
public class redBottomParkOnly extends LinearOpMode {
    robot bot = new robot();
    @Override
    public void runOpMode(){
        waitForStart();
        bot.init(hardwareMap, this);
        bot.moveStraight(48, 0.8, 1);
        bot.strafe(48, -1, 0.8);

    }
}
