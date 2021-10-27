package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

//keshav says hi

@TeleOp (name = "drive", group = "sus")
public class drive extends LinearOpMode {
    robot bot = new robot();
    @Override
    public void runOpMode(){
        bot.init(hardwareMap, this);
        waitForStart();
        double lx = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double ly = -gamepad1.left_stick_y;
        while(opModeIsActive()) {
            double lf = ly + rx + lx;
            double lb = ly + rx - lx;
            double rf = ly - rx - lx;
            double rb = ly - rx + lx;

            double ratio;
            double max = Math.max(Math.max(Math.abs(lb), Math.abs(lf)), Math.max(Math.abs(rb), Math.abs(rf)));
            double magnitude = Math.sqrt((lx * lx) + (ly * ly) + (rx * rx));
            ratio = .8 * magnitude / max;
            if (max == 0) {
                ratio = 0;
            }

            bot.leftFront.setPower(lf * ratio);
            bot.leftBack.setPower(lb * ratio);
            bot.rightFront.setPower(rf * ratio);
            bot.rightBack.setPower(rb * ratio);

            if (gamepad1.left_bumper) {
                bot.spin.setPower(0.8);
            } else {
                bot.spin.setPower(0);
            }
            if (gamepad1.right_bumper) {
                bot.collect.setPower(0.8);
            } else {
                bot.collect.setPower(0);
            }
        }

    }


}
