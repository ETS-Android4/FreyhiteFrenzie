package org.firstinspires.ftc.teamcode.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "blueBottomArm", group = "a")
public class blueBottomArm extends LinearOpMode {

    int barcode;
    OpenCvWebcam webcam;
    @Override
    public void runOpMode() throws InterruptedException {
    robot bot = new robot(hardwareMap, this);
        bot.initOpenCV(webcam);
        waitForStart();
//        bot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        bot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        bot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        bot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        bot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        bot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        bot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        bot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.resetEncoders();
        //sleep(2000);
        bot.strafe(17,1, 0.4);

        //bot.moveStraight(13, 0.4, -1);
        //insertOpenCV stuff here!

        //read the barcode and set to int barcode
        barcode = 1;
        if (barcode == 1) {
            bot.moveStraight(15, 0.4, -1);
            bot.armToTop();
            sleep(1000);
            bot.wristDrop();
            sleep(1000);
            bot.resetEncoders();
            bot.moveStraight(3, 0.4,1);
            bot.armReset();
        }
        else if (barcode == 2) {
            bot.moveStraight(12, 0.4, -1);
            bot.armToMiddle();
            sleep(1000);
            bot.wristDrop();
            sleep(1000);
            bot.wristReset();
            //bot.moveStraight(2, 0.4, 1);
            bot.armReset();
        }
        else {
            bot.moveStraight(9, 0.4, -1);
            bot.armToBottom();
            sleep(1000);
            bot.wristDrop();
            sleep(1000);
            bot.armReset();
            bot.moveStraight(1, 0.4, -1);
        }



        //move back
        //bot.moveStraight(5, 0.4, 1);
        //sleep(500);
        //bot.moveStraight(2, .4, -1);
        sleep(500);
        bot.turnTo(-Math.PI/2 + 0.4, .4); //change 0.1
        sleep(500);
        // positions the bot next to the carousel
        bot.moveStraight(54, 0.4, -1);
        //bot.strafe(3,1, 0.4);
        sleep(500);
        bot.resetEncoders();
       // bot.strafe(3, -1, 0.25);
        // spins the carousel motor
        bot.spinCarousel(bot.DIRECTION);
        sleep(5000);
        bot.spinCarousel(0);

        // parks the robot
//        bot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        bot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.moveStraight(3, 0.4, 1);
        bot.strafe(13, 1, 0.3);
        //bot.moveStraight(2,.3,-1);

    }
}
