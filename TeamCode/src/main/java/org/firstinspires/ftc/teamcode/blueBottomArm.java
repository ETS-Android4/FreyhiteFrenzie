package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "blueBottomArm", group = "a")
public class blueBottomArm extends LinearOpMode {

    robot bot = new robot();
    int barcode;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        bot.init(hardwareMap, this);
//        bot.initOpenCV();
        bot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.strafe(17.5,1, 0.4);
        bot.moveStraight(16, 0.45, -1);
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
        bot.moveStraight(5, 0.4, 1);
        sleep(500);
        //bot.moveStraight(2, .4, -1);
        sleep(1000);
        bot.turnTo(-Math.PI/2 + 0.1, .4); //change 0.1

        // positions the bot next to the carousel
        bot.moveStraight(48, 0.4, -1);
        sleep(1000);

        // spins the carousel motor
        bot.spinCarousel(bot.direction);
        sleep(5000);
        bot.spin.setPower(0);

        // parks the robot
        bot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.strafe(16, 1, 0.3);
        bot.moveStraight(2,.3,-1);

    }
}
