package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "redBottom", group = "d")
public class redBottom extends LinearOpMode {
    robot bot = new robot();
    @Override
    public void runOpMode(){
        waitForStart();
        bot.init(hardwareMap, this);
        bot.strafe(8, -1, 0.3);
        bot.spin.setPower(0.6);
        sleep(5000);
        bot.spin.setPower(0);
        bot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.moveStraight(20, 0.4, 1);
        bot.strafe(10,-1,0.3);

    }
}
