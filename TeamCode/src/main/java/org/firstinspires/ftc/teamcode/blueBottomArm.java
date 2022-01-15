package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "blueBottomArm", group = "a")
public class blueBottomArm extends LinearOpMode {

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
        bot.strafe(17.5,1, 0.4);
        //bot.moveStraight(13, 0.4, -1);
        //insertOpenCV stuff here!

        //read the barcode and set to int barcode
        barcode = 2;
        if (barcode == 1) {
            bot.moveStraight(13, 0.4, -1);
            bot.armToTop();
            sleep(1000);
            bot.wristDrop();
            sleep(1000);
            bot.armReset();
            bot.moveStraight(5, 0.4,1);
        }
        else if (barcode == 2) {
            bot.moveStraight(3.65, 0.4, -1);
            bot.armToMiddle();
            sleep(1000);
            bot.wristDrop();
            sleep(1000);
            bot.moveStraight(2, 0.4, 1);
            bot.armReset();
        }
        else {
            bot.moveStraight(1.5, 0.4, -1);
            bot.armToBottom();
            sleep(1000);
            bot.wristDrop();
            sleep(1000);
            bot.armReset();
            bot.moveStraight(1, 0.4, -1);
        }

        bot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //move back
        //bot.moveStraight(5, 0.4, 1);
        sleep(500);
        //bot.moveStraight(2, .4, -1);
        sleep(1000);
        bot.turnTo(-Math.PI/2 + 0.1, .4); //change 0.1

        // positions the bot next to the carousel
        bot.moveStraight(48, 0.4, -1);
        bot.strafe(3,-1, 0.4);
        sleep(1000);

        // spins the carousel motor
        bot.spinCarousel(bot.direction);
        sleep(5000);
        bot.spin.setPower(0);

        // parks the robot
        bot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.turnTo(Math.PI, .4);
        bot.strafe(21, 1, 0.3);
        //bot.moveStraight(2,.3,-1);

    }
}
