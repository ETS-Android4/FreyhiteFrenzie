package org.firstinspires.ftc.teamcode.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "redBottomArm", group = "f")
public class redBottomArm extends LinearOpMode {

    //line up left (techically right cause robot is facing wall) side of robot 18 degrees from left side of mat (should look like an upside down V)
    //line up bottom left wheel (technically top right) 6 inches away from left edge of the mat

    int barcode;
    OpenCvWebcam webcam;
    @Override
    public void runOpMode() throws InterruptedException {
    robot bot = new robot(hardwareMap, this);
        bot.initOpenCV(webcam);
        waitForStart();
        bot.resetEncoders();
        //sleep(7000);
        bot.strafe(17.5,-1, 0.4);

        //insertOpenCV stuff here!

        //read the barcode and set to int barcode
        barcode = 1;
        if (barcode == 1) {
            bot.moveStraight(16, 0.4, -1);
            bot.armToTop();
            sleep(1000);
            bot.wristDrop();
            sleep(1000);
            bot.armReset();
            bot.resetEncoders();
            bot.moveStraight(2, 0.4,1);
        }
        else if (barcode == 2) {
            bot.moveStraight(10, 0.4, -1);
            bot.armToMiddle();
            sleep(1000);
            bot.wristDrop();
            sleep(1000);
            bot.armReset();
            bot.resetEncoders();
            //bot.moveStraight(2, 0.4, 1);
        }
        else if (barcode == 3){
            bot.resetEncoders();
            bot.moveStraight(9, 0.4, -1);
            bot.armToBottom();
            sleep(1000);
            bot.wristDrop();
            sleep(1000);
            bot.armReset();
            bot.resetEncoders();
            //bot.moveStraight(6, 0.4, 1);
        }
        bot.resetEncoders();
        //move back
        bot.moveStraight(4, 0.5, 1);
        sleep(1000);
        bot.turnTo(Math.PI/2, 0.4);
        //bot.strafe(20, 1, 0.4);
        bot.moveStraight(54, 0.4, -1);
//        bot.leftBack.setPower(-0.3);
//        bot.leftFront.setPower(-0.3);
//        bot.rightBack.setPower(0.3);
//        bot.rightFront.setPower(0.3);
//        sleep(550);
//        bot.leftBack.setPower(0);
//        bot.leftFront.setPower(0);
//        bot.rightFront.setPower(0);
//        bot.rightBack.setPower(0);
        //sleep(1000);
        //bot.turnTo(Math.PI - 0.1, 0.4);
        bot.resetEncoders();
        sleep(1000);
        bot.strafe(2,-1, 0.25);
        telemetry.addData("go:", true);
        // positions the bot next to the carousel
        //bot.moveStraight(2, 0.4, -1);
       // bot.strafe(7, -1,0.4);
        bot.resetEncoders();
        //bot.strafe(6, -1, 0.3);
        // spin the carousel
        bot.spinCarousel(-bot.DIRECTION);
        sleep(5000);
        bot.spinCarousel(0);
        bot.resetEncoders();
        // parks the robot
        bot.moveStraight(21, 0.5, 1);
        bot.strafe(9, -1, 0.4);
//        bot.moveStraight(5, 0.4,-1);


    }
}