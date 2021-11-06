package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "blueBottom", group = "a")
public class blueBottom extends LinearOpMode {
    robot bot = new robot();
    @Override
    public void runOpMode(){
        waitForStart();
        bot.init(hardwareMap, this);
        bot.moveStraight(4, 0.4, 1);
        bot.turnTo(Math.PI/2, 0.4);
//        bot.moveStraight(15, 0.4, -1);
//        bot.spin.setPower(0.6);
//        sleep(4000);
//        bot.spin.setPower(0);
//        bot.strafe(48, 1, 0.4);
//        bot.moveStraight(4, 0.4, -1);


    }

}
