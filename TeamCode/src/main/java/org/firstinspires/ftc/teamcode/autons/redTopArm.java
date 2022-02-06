package org.firstinspires.ftc.teamcode.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "redTopArm", group = "a")
public class redTopArm extends LinearOpMode {

    int barcode;
    OpenCvWebcam webcam;

    @Override
    public void runOpMode() throws InterruptedException {
    robot bot = new robot(hardwareMap, this);
        bot.initOpenCV(webcam);
        waitForStart();
        bot.resetEncoders();
        sleep(2000);
        bot.strafe(15,1, 0.4);


        //insertOpenCV stuff here!

        //read the barcode and set to int barcode
        barcode = 1;
        if (barcode == 1) {
            bot.moveStraight(14.5, 0.5, -1);
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
        bot.resetEncoders();
        bot.moveStraight(15, 0.5, 1);
        sleep(500);
        bot.turnTo(Math.PI/2, 0.4);
        // park
        bot.strafe(6, 1, 0.4);
        bot.moveStraight(45, 0.4, 1);
    }

}
