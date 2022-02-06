package org.firstinspires.ftc.teamcode.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "blueTopArm", group = "f")
public class blueTopArm extends LinearOpMode {

    int barcode;
    OpenCvWebcam webcam;

    @Override
    public void runOpMode() throws InterruptedException {
    robot bot = new robot(hardwareMap, this);
        bot.initOpenCV(webcam);
        waitForStart();
        bot.resetEncoders();
        bot.strafe(14,-1, 0.4);
        bot.moveStraight(15.5, 0.5, -1);

        //insertOpenCV stuff here!

        //read the barcode and set to int barcode
        barcode = 1;
        if (barcode == 1) {
            bot.armToTop();
        }
        else if (barcode == 2) {
            bot.armToMiddle();
        }
        else if (barcode == 3){
            bot.armToBottom();
        }
        sleep(2000);
        bot.wristDrop();
        sleep(1000);
        bot.armReset();
        bot.resetEncoders();
        //move back
        bot.moveStraight(15, 0.5, 1);
        sleep(500);
        bot.turnTo(Math.PI/2, 0.4);
        // positions the bot next to the carousel
        bot.moveStraight(16, 0.4, -1);
        bot.strafe(10, 1, 0.4);
        bot.moveStraight(32, 0.5, -1);

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