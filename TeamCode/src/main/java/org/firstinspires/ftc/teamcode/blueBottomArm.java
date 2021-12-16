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
        bot.initOpenCV();
        bot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.strafe(24,1, 0.4);
        bot.moveStraight(10, 0.5, -1);
        //insertOpenCV stuff here!

        //read the barcode and set to int barcode

        if (barcode == 3) {
            bot.armTo3();
        }
        else if (barcode == 2) {
            bot.armTo2();
        }
        else {
            bot.armTo1();
        }
        //bot.wristDrop1();
        bot.armReset();

        bot.init(hardwareMap, this);
        bot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //move back
        bot.moveStraight(10, 0.5, 1);
        bot.moveStraight(2, .45, -1);
        bot.turnTo(-Math.PI/2, .45);

        // positions the bot next to the carousel
        bot.moveStraight(30, 0.4, -1);
        sleep(1000);

        // spins the carousel motor
        bot.spinCarousel(bot.direction);
        sleep(5000);
        bot.spin.setPower(0);

        // parks the robot
        bot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.strafe(30, 1, 0.3);
        bot.moveStraight(4,.3,-1);

    }
}
