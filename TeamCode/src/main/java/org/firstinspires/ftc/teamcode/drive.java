package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


@TeleOp (name = "TrulyIntelligentTeleopSoftware", group = "sus")
public class drive extends LinearOpMode {

     // initialize robot
    @Override
    public void runOpMode() throws InterruptedException {
        robot bot = new robot(hardwareMap, this);
        waitForStart();

        double lx, rx, ly; // intialize variables for the gamepad
        boolean in = true;
        byte wristExt = 0; //0 fully ext, 1 half ext, 2 fully ext
        byte armExt = 0; //0 reset, 1 half ext, 2 fully ext
        boolean wristExtLate = false;
        boolean armExtLate1 = false;
        boolean armExtLate2 = false;
        boolean armExtLate3 = false;
        boolean slowModeLate = false;
        double slowMode = 1.0;
        boolean slow = false;
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

            // sets the motor power
            bot.leftFront.setPower(lf * ratio * 0.8 * slowMode);
            bot.leftBack.setPower(lb * ratio * 0.8 * slowMode);
            bot.rightFront.setPower(rf * ratio * 0.8 * slowMode);
            bot.rightBack.setPower(rb * ratio * 0.8 * slowMode);
            // reset leftFront encoder
            if (gamepad1.a){
                bot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                bot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            // motor powers are slowed



            // turns on the carousel motor
            if (gamepad2.left_bumper) {
                bot.spinCarousel(1);
            }
            else if (gamepad2.right_bumper){
                bot.spinCarousel(-1);
            }
            else{
                bot.spin.setPower(0);
            }
            if (gamepad1.y && !slowModeLate) {
                if (!slow) {
                    slowMode = 0.5;
                    slow = true;
                }
                else{
                    slowMode = 1.0;
                    slow = false;
                }
            }

            //servo controls for arm
            //wrist:
            if(gamepad2.a && !wristExtLate)
            {
                if(wristExt == 1)
                {
                    bot.wristReset();
                    wristExt = 0;
                }
                else if(wristExt == 0)
                {
                    bot.wristDrop();
                    wristExt = 1;
                }
            }

            telemetry.addData("Wrist Extension: ", wristExt);

            //arm:
            if(gamepad2.y && !armExtLate1)
            {
                wristExt = 0;
                if(armExt != 0)
                {
                    bot.armReset();
                    armExt = 0;
                }
                else if(armExt == 0)
                {
                    bot.armToTop();
                    armExt = 1;
                }
            }
            if(gamepad2.x && !armExtLate2)
            {
                wristExt = 0;
                if(armExt != 0)
                {
                    bot.armReset();
                    armExt = 0;
                }
                else if(armExt == 0)
                {
                    bot.armToMiddle();
                    armExt = 2;
                }
            }
            if(gamepad2.b && !armExtLate3)
            {
                wristExt = 0;
                if(armExt != 0)
                {
                    bot.armReset();
                    armExt = 0;
                }
                else if(armExt == 0)
                {
                    bot.armToBottom();
                    armExt = 3;
                }
            }

            //put values for late booleans
            wristExtLate = gamepad2.a;
            armExtLate1 = gamepad2.y;
            armExtLate2 = gamepad2.x;
            armExtLate3 = gamepad2.b;
            slowModeLate = gamepad1.y;

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
            //telemetry.addData("carousel", bot.spin.getPower());
            //telemetry.addData("leftFront:", bot.rightBack.getPower());
//            telemetry.addData("lf",lf);
//            telemetry.addData("rf", rf);
//            telemetry.addData("lb", lb);
//            telemetry.addData("rb",rb);
            for (int i = 0; i<robot.encoderMotors.length; i++)
            {
                telemetry.addData("encodermotors" + i + ": ", robot.encoderMotors[i].getCurrentPosition());
            }
            telemetry.addData("Average Encoder Motors: ", Util.getAverageEncoderPosition(robot.encoderMotors));
            telemetry.addData("rightFront:", bot.rightFront.getCurrentPosition());
            //telemetry.addData("leftSticky", gamepad1.left_stick_y);
            telemetry.addData("wrist: ", bot.wrist.getPosition());
            telemetry.addData("arm: ", bot.arm.getPosition());
            telemetry.addData("leftFront: ", bot.leftFront.getCurrentPosition());
            telemetry.addData("lf", bot.leftFront.getPortNumber());
            telemetry.addData("lb", bot.leftBack.getPortNumber());
            telemetry.addData("rf", bot.rightFront.getPortNumber());
            telemetry.addData("rb", bot.rightBack.getPortNumber());
            //telemetry.addData("in: ", in);
            telemetry.update();
        }
    }
}
