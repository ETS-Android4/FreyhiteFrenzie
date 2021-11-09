package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

//keshav says hi

@TeleOp (name = "drive", group = "sus")
public class drive extends LinearOpMode {

    robot bot = new robot(); // initialize robot

    @Override
    public void runOpMode(){

        bot.init(hardwareMap, this);
        waitForStart();

        double lx, rx, ly; // intialize variables for the gamepad

        while(opModeIsActive()) {

            // set the gamepad variables
            lx = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;
            ly = -gamepad1.left_stick_y;

            // arithmatic to get motor values not scaled
            double lf = ly + rx + lx;
            double lb = ly + rx - lx;
            double rf = ly - rx - lx;
            double rb = ly - rx + lx;

            // scale the motor values
            double ratio;
            double max = Math.max(Math.max(Math.abs(lb), Math.abs(lf)), Math.max(Math.abs(rb), Math.abs(rf)));
            double magnitude = Math.sqrt((lx * lx) + (ly * ly) + (rx * rx));
            ratio = .8 * magnitude / max;
            if (max == 0) {
                ratio = 0;
            }

            // reset rightFront encoder
            if (gamepad1.a){
                bot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                bot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            // sets the motor power
            bot.leftFront.setPower(lf * ratio * 0.8);
            bot.leftBack.setPower(lb * ratio * 0.8);
            bot.rightFront.setPower(rf * ratio * 0.8);
            bot.rightBack.setPower(rb * ratio * 0.8);

            // adds telemetry data
            telemetry.addData("heading", bot.imu.getAngularOrientation().firstAngle);
            telemetry.addData("leftFront:", bot.rightBack.getPower());
            telemetry.addData("leftSticky", gamepad1.left_stick_y);
            telemetry.addData("rightFront:",bot.rightFront.getCurrentPosition());
            telemetry.update();

            // turns on the carousel motor
            if (gamepad2.left_bumper) {
                bot.spin.setPower(0.6);
            }
            else {
                bot.spin.setPower(0);
            }

            // turns on the carousel motor in the other direction
            if (gamepad2.right_bumper) {
                bot.spin.setPower(-0.6);
            }
            else{
                bot.spin.setPower(0);
            }

            // operates the servo motor
            if (gamepad2.b)
            {
                bot.collection.setPosition(0.4);
                telemetry.addData("Collection:", bot.collection.getPosition());
                telemetry.update();
            }
            else if (gamepad2.y)
            {
                bot.collection.setPosition(0.75);
                telemetry.addData("Collection:", bot.collection.getPosition());
                telemetry.update();
            }
        }
    }
}
