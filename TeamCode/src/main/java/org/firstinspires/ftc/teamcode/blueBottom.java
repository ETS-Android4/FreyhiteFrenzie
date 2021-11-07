package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "blueBottom", group = "a")
public class blueBottom extends LinearOpMode {
    robot bot = new robot();
    @Override
    public void runOpMode(){
        waitForStart();
        bot.init(hardwareMap, this);
        bot.moveStraight(2, 0.4, 1);
        sleep(1000);
        turnTo(Math.PI/2, 0.35);
        bot.moveStraight(14,0.3,-1);
        bot.spin.setPower(0.6);
        sleep(5000);
        bot.spin.setPower(0);
        bot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.strafe(13, 1, 0.3);
        bot.moveStraight(4,.3,-1);


    }
    public void turnTo(double angle, double speed){
        int direction = (int)((bot.imu.getAngularOrientation().firstAngle-angle)/Math.abs(bot.imu.getAngularOrientation().firstAngle-angle));
        while (Math.abs(angle - bot.imu.getAngularOrientation().firstAngle) > 0.04){
            bot.leftFront.setPower(direction * speed);
            bot.leftBack.setPower(direction * speed);
            bot.rightFront.setPower(-direction * speed);
            bot.rightBack.setPower(-direction * speed);
            bot.linearOpMode.telemetry.addData("angle", bot.imu.getAngularOrientation().firstAngle);
            bot.linearOpMode.telemetry.addData("current-goal", Math.abs(bot.imu.getAngularOrientation().firstAngle - angle));
            bot.linearOpMode.telemetry.update();
        }
        bot.leftFront.setPower(0);
        bot.leftBack.setPower(0);
        bot.rightFront.setPower(0);
        bot.rightBack.setPower(0);
        bot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
