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
        bot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.init(hardwareMap, this);

        bot.strafe(18,-1, 0.4);
        bot.moveStraight(17, 0.5, -1);

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
        bot.moveStraight(16, 0.5, 1);
        bot.strafe(40, 1, 0.4);

        // positions the bot next to the carousel
        sleep(500);
        bot.turnTo(Math.PI, 0.5);
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
        bot.moveStraight(7, 0.6, 1);
        bot.strafe(14, -1, 0.4);

    }
}