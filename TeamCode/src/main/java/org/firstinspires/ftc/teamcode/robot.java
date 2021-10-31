package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class robot {
    DcMotor leftBack, leftFront, rightBack, rightFront;
    DcMotor spin, collect;
    BNO055IMU imu;
    Orientation currentAngle;
    public void init(HardwareMap hardwareMap, LinearOpMode linearOpMode){
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        spin = hardwareMap.get(DcMotor.class, "spin");
        collect = hardwareMap.get(DcMotor.class, "collect");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.getAngularOrientation();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        currentAngle = imu.getAngularOrientation();
    }
    public void moveStraight(double inches, double speed, double ticksToInch, double direction){
        while ((double)leftFront.getCurrentPosition() * direction/ ticksToInch <= inches){
            leftFront.setPower(direction * speed);
            leftBack.setPower(direction* speed);
            rightFront.setPower(direction * speed);
            rightBack.setPower(direction * speed);
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void strafe(double inches, int direction, double speed, double ticksToInch){
            while ((double)leftFront.getCurrentPosition() / ticksToInch <= (double)inches){
                leftFront.setPower(direction * speed);
                leftBack.setPower(-direction * speed);
                rightFront.setPower(-direction * speed);
                rightBack.setPower(direction * speed);
            }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //if direction is 1, strafe right, if direction is -1 strafe left
        //strafe a certain number of inches
    }
    public void turnTo(double angle, double speed, int direction){
        while (currentAngle.firstAngle <= angle){
            leftFront.setPower(direction * speed);
            leftBack.setPower(direction * speed);
            rightFront.setPower(-direction * speed);
            rightBack.setPower(-direction * speed);
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}
