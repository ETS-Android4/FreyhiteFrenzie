package org.firstinspires.ftc.teamcode;

import android.os.DropBoxManager;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

public class robot {
    DcMotor leftBack, leftFront, rightBack, rightFront;
    DcMotor spin; //collect;
    BNO055IMU imu;
    Orientation currentAngle;
    LinearOpMode linearOpMode;
    HardwareMap hardwareMap;
    final static double TICKS_TO_INCH_FORWARD = 36.87;
    final static double TICKS_TO_INCH_STRAFE = 46.68;
    public void init(HardwareMap hardwareMap, LinearOpMode linearOpMode){
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        spin = hardwareMap.get(DcMotor.class, "spin");
//        collect = hardwareMap.get(DcMotor.class, "collect");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.getAngularOrientation();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        imu.initialize(parameters);
        currentAngle = imu.getAngularOrientation();
        this.linearOpMode = linearOpMode;
        this.hardwareMap = hardwareMap;
    }
    public void moveStraight(double inches, double speed, double direction){
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while ((double)rightFront.getCurrentPosition() * direction/TICKS_TO_INCH_FORWARD <= inches){
            leftFront.setPower(direction * speed);
            leftBack.setPower(direction* speed);
            rightFront.setPower(direction * speed);
            rightBack.setPower(direction * speed);
            linearOpMode.telemetry.addData("rightFront:",rightFront.getCurrentPosition());
            linearOpMode.telemetry.update();
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

    }
    public void strafe(double inches, int direction, double speed){
            while ((double)rightFront.getCurrentPosition() / TICKS_TO_INCH_STRAFE <= (double)inches){
                leftFront.setPower(direction * speed);
                leftBack.setPower(-direction * speed);
                rightFront.setPower(-direction * speed);
                rightBack.setPower(direction * speed);
                linearOpMode.telemetry.addData("rightFront:", rightFront.getCurrentPosition());
            }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        //if direction is 1, strafe right, if direction is -1 strafe left
        //strafe a certain number of inches
    }
    public void turnTo(double angle, double speed){
        int direction = (int)((imu.getAngularOrientation().firstAngle-angle)/Math.abs(imu.getAngularOrientation().firstAngle-angle));
        while (angleWrap(Math.abs(angle - imu.getAngularOrientation().firstAngle)) > 0.06){
            leftFront.setPower(direction * speed);
            leftBack.setPower(direction * speed);
            rightFront.setPower(-direction * speed);
            rightBack.setPower(-direction * speed);
            linearOpMode.telemetry.addData("angle", imu.getAngularOrientation().firstAngle);
            linearOpMode.telemetry.addData("current-goal", Math.abs(imu.getAngularOrientation().firstAngle - angle));
            linearOpMode.telemetry.update();
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
    public static double angleWrap(double angle){
        while(angle>Math.PI){
            angle-=2*Math.PI;
        }
        while(angle<-Math.PI){
            angle+=2*Math.PI;
        }
        return angle;
    }
}
