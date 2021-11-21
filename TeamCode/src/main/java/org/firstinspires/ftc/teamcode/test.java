package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "test", group = "ad")
public class test extends LinearOpMode {
//for testing purposes
    robot bot = new robot();
    @Override
    public void runOpMode() {
        waitForStart();
        bot.init(hardwareMap, this);
        sleep(1000);
        bot.wrist.setPosition(1);
        sleep(1000);
        //bot.wrist.setPosition(0);
    }
}
