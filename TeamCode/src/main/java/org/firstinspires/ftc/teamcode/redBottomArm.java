package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "redBottomArm", group = "f")
public class redBottomArm extends LinearOpMode {

    robot bot = new robot();
    int barcode;

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        bot.init(hardwareMap, this);
        bot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.init(hardwareMap, this);

        bot.strafe(24,-1, 0.4);
        bot.moveStraight(10, 0.5, -1);

        //insertOpenCV stuff here!

        //read the barcode and set to int barcode

        if (barcode == 3) {
            bot.armTo3();
        }
        else if (barcode == 2) {
            bot.armTo2();
        }
        else {
            bot.armTo1();
        }
        //bot.wristDrop();
        bot.armReset();

        //move back
        bot.moveStraight(10, 0.5, 1);

        // positions the bot next to the carousel
        bot.moveStraight(2, 0.5, -1);
        bot.turnTo(Math.PI, 0.45);
        bot.strafe(14, -1, 0.25);

        // spin the carousel
        bot.spinCarousel(-bot.direction);
        sleep(5000);
        bot.spin.setPower(0);

        // parks the robot
        bot.strafe(12, 1, 0.6);
        bot.moveStraight(6, 0.3, 1);

    }
}