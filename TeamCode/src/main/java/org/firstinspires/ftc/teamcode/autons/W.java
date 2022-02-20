package org.firstinspires.ftc.teamcode.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot;

@Autonomous(name = "W", group = "cc")
public class W extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        robot bot = new robot(hardwareMap, this);
        waitForStart();
        bot.setMotorPowers(-0.8, -0.8, 0.8, 0.8);
        while (opModeIsActive()){
            bot.flash();
            telemetry.speak("doodoo");
        }
    }
}
