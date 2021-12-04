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
    public void runOpMode() throws InterruptedException {

        bot.init(hardwareMap, this);
        waitForStart();

        double lx, rx, ly; // intialize variables for the gamepad
        boolean in = true;
        boolean wristExt = false;
        boolean wristExtLate = false;
        boolean armExt = false;
        boolean armExtLate = false;
        while(opModeIsActive()) {

            // set the gamepad variables
            lx = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;
            ly = -gamepad1.left_stick_y;

            // arithmetic to get motor values - not scaled
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

            // turns on the carousel motor
            if (gamepad2.left_bumper) {
                bot.spinCarousel(1);
            }
            else {
                bot.spin.setPower(0);
            }

            // turns on the carousel motor in the other direction
            if (gamepad2.right_bumper) {
                bot.spinCarousel(-1);
            }
            else{
                bot.spin.setPower(0);
            }

            //servo controls for arm
            if(!gamepad2.b && wristExtLate)
            {
                if(wristExt)
                {
                    bot.wrist.setPosition(0);
                    wristExt = false;
                }
                else{
                    bot.wrist.setPosition(1);
                    wristExt = true;
                }
            }
            if(!gamepad2.x && armExtLate)
            {
                if(armExt)
                {
                    bot.arm.setPosition(0.4);
                    armExt = false;
                }
                else
                {
                    bot.arm.setPosition(0.74);
                    armExt = true;
                }
            }

            //put values for late booleans
            wristExtLate = gamepad2.b;
            armExtLate = gamepad2.x;

            //collection controls
            if(gamepad2.left_trigger > 0.5){
                bot.intake.setPower(0.8);
            }
            else if(gamepad2.right_trigger > 0.5){
                bot.intake.setPower(-0.8);
            }
            else {
                bot.intake.setPower(0);
            }
            // adds telemetry data
            telemetry.addData("heading", bot.imu.getAngularOrientation().firstAngle);
            telemetry.addData("heading3", bot.imu.getAngularOrientation().thirdAngle);
            //range: front wheels off ground 0.08 to -1.5
            //back wheels off ground 0.08 to 1.5
            telemetry.addData("leftFront:", bot.rightBack.getPower());
            telemetry.addData("leftSticky", gamepad1.left_stick_y);
            telemetry.addData("wrist:", bot.wrist.getPosition());
            telemetry.addData("arm: ", bot.arm.getPosition());
            telemetry.addData("rightFront:",bot.rightFront.getCurrentPosition());
            telemetry.addData("in: ", in);
            telemetry.update();
        }
    }
}
