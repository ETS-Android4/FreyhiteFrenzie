package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "redTopArm", group = "a")
public class redTopArm extends LinearOpMode {

    robot bot = new robot();
    int barcode;
    OpenCvWebcam webcam;

    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap, this);
        bot.OpenCV(webcam);
        waitForStart();
        bot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        bot.strafe(10,1, 0.4);
        bot.moveStraight(20, 0.5, -1);

        //insertOpenCV stuff here!

        //read the barcode and set to int barcode
        barcode = 3;
        if (barcode == 1) {
            bot.armToTop();
        }
        else if (barcode == 2) {
            bot.armToMiddle();
        }
        else {
            bot.armToBottom();
        }
        sleep(2000);
        bot.wristDrop();
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
        bot.strafe(6, -1, 0.4);
        bot.moveStraight(32, 0.6, -1);



    }

}
