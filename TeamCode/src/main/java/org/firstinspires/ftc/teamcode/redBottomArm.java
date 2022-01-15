package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "redBottomArm", group = "f")
public class redBottomArm extends LinearOpMode {

    //line up left (techically right cause robot is facing wall) side of robot 18 degrees from left side of mat (should look like an upside down V)
    //line up bottom left wheel (technically top right) 6 inches away from left edge of the mat

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
        bot.init(hardwareMap, this);

        bot.strafe(20,-1, 0.4);

        //insertOpenCV stuff here!

        //read the barcode and set to int barcode
        barcode = 3;
        if (barcode == 1) {
            bot.moveStraight(13, 0.4, -1);
            bot.armToTop();
            sleep(1000);
            bot.wristDrop();
            sleep(1000);
            bot.armReset();
            bot.moveStraight(2, 0.4,1);
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
        else if (barcode == 3){
            bot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bot.moveStraight(8.8, 0.4, -1);
            bot.armToBottom();
            sleep(1000);
            bot.wristDrop();
            sleep(1000);
            bot.armReset();
            //bot.moveStraight(6, 0.4, 1);
        }
        bot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //move back
        bot.moveStraight(4, 0.5, 1);
        bot.strafe(20, 1, 0.4);
        bot.moveStraight(4, 0.5, -1);
        bot.strafe(20, 1, 0.4);

        // positions the bot next to the carousel
        sleep(500);
        bot.turnTo((Math.PI/2) + 0.5, 0.5);
        sleep(1000);
        //bot.turnTo(Math.PI - 0.1, 0.45);
        bot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //bot.strafe(6, -1, 0.3);
        sleep(1000);
        // spin the carousel
        bot.spinCarousel(-bot.direction);
        sleep(5000);
        bot.spin.setPower(0);

        // parks the robot
        bot.moveStraight(8, 0.6, 1);
        bot.strafe(15, -1, 0.4);

    }
}