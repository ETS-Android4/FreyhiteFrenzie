package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Util {
    public static double getAverageEncoderPosition(DcMotor[] dm)
    {
        double sum = 0;
        for(int i = 0; i<dm.length; i++)
        {
            sum += Math.abs(dm[i].getCurrentPosition());
        }
        return sum/dm.length;
    }

    public static void resetAllEncoders(DcMotor[] dm)
    {
        for(int i = 0; i<dm.length; i++)
        {
            dm[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dm[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public static double angleWrap(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }

    public static void setMotors(double magnitude, int lfD, int lbD, int rfD, int rbD, robot bot)
    {
        bot.leftFront.setPower(magnitude * lfD);
        bot.leftBack.setPower(magnitude * lbD);
        bot.rightFront.setPower(magnitude * rfD);
        bot.rightBack.setPower(magnitude * rbD);
    }

    public static void setMotors(double magnitude, robot bot){
        bot.leftFront.setPower(magnitude);
        bot.leftBack.setPower(magnitude);
        bot.rightFront.setPower(magnitude);
        bot.rightBack.setPower(magnitude);
    }
}
