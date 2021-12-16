package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "redTopArm", group = "a")
public class redTopArm extends LinearOpMode {

    robot bot = new robot();
    int barcode;

    @Override
    public void runOpMode() throws InterruptedException {
        bot.initOpenCV();
        waitForStart();
        bot.init(hardwareMap, this);
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
        //bot.wristDrop();
        bot.armReset();



        //move back
        bot.moveStraight(10, 0.5, 1);

        // positions the bot next to the carousel
        bot.moveStraight(2,0.5,-1);
        bot.turnTo(Math.PI/2, 0.45);
        sleep(1000);
        bot.strafe(20,-1,0.4);

        // spins the carousel motor
        bot.spinCarousel(bot.direction);
        sleep(5000);
        bot.spin.setPower(0);

        // parks the robot
        bot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.strafe(10, 1, 0.3);
        bot.moveStraight(4,.3,1);


    }

}
