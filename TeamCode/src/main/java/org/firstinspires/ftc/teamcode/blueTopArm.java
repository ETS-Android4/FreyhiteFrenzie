package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "blueTopArm", group = "f")
public class blueTopArm extends LinearOpMode {

    robot bot = new robot();
    int barcode;

    @Override
    public void runOpMode() throws InterruptedException {


        bot.init(hardwareMap, this);
        bot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //bot.initOpenCV();
        waitForStart();
        bot.strafe(10,-1, 0.4);
        bot.moveStraight(20, 0.5, -1);

        //insertOpenCV stuff here!

        //read the barcode and set to int barcode
        barcode = 3;
        if (barcode == 3) {
            bot.armTo1();
        }
        else if (barcode == 2) {
            bot.armTo2();
        }
        else {
            bot.armTo1();
        }
        sleep(2000);
        bot.wristDrop2();
        sleep(1000);
        bot.armReset();
        bot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //move back
        bot.moveStraight(15, 0.5, 1);
        sleep(500);
        bot.turnTo(Math.PI/2, 0.4);
        // positions the bot next to the carousel
        bot.moveStraight(20, 0.4, -1);
        bot.strafe(6, 1, 0.4);
        bot.moveStraight(32, 0.6, -1);

        //sleep(1000);
        //bot.turnTo(0, 0.45);
        //bot.moveStraight(45, 0.4, -1);

        // spins the carousel motor
//        bot.spinCarousel(bot.direction);
//        sleep(6000);
//        bot.spin.setPower(0);

        // parks the robot
//        bot.strafe(12, 1, 0.6);
//        bot.moveStraight(6, 0.3, -1);

    }
}