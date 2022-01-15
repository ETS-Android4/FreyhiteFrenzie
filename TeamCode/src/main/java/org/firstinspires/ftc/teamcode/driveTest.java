package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

    @TeleOp(name = "driveTest", group = "sus")
    public class driveTest extends LinearOpMode {

        robotTest arm = new robotTest(); // initialize robot

        @Override
        public void runOpMode() throws InterruptedException {

            arm.init(hardwareMap, this);

            waitForStart();
            while(opModeIsActive()) {
                if (gamepad1.x) {
                    arm.arm.setPosition(1);
                }
                if (gamepad1.y) {
                    arm.arm.setPosition(0);
                }
                if (gamepad1.a) {
                    arm.motor.setPower(0.8);
                }
                if (!gamepad1.a) {
                    arm.motor.setPower(0);
                }
            }
        }
        //delete this section later, it's for testing servos


    }

