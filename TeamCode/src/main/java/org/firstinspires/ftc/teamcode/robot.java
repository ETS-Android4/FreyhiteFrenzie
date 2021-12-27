package org.firstinspires.ftc.teamcode;

import android.os.DropBoxManager;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class robot {
    DcMotor leftBack, leftFront, rightBack, rightFront;
    DcMotor spin;
    DcMotor intake;
    //Servo collection;//collect;
    Servo wrist;
    Servo arm;

    BNO055IMU imu;
    Orientation currentAngle;
    OpenCvPipeline pipeline;
    OpenCvCamera camera;
    LinearOpMode linearOpMode;
    HardwareMap hardwareMap;
    WebcamName webcamName;

    int direction;

    final static double TICKS_TO_INCH_FORWARD = 36.87;
    final static double TICKS_TO_INCH_STRAFE = 70.68;

    public void init(HardwareMap hardwareMap, LinearOpMode linearOpMode){
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        //collection = hardwareMap.get(Servo.class, "collection");
        spin = hardwareMap.get(DcMotor.class, "spin");
        intake = hardwareMap.get(DcMotor.class, "intake");
        wrist = hardwareMap.get(Servo.class, "wrist");
        arm = hardwareMap.get(Servo.class, "arm");


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.getAngularOrientation();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        currentAngle = imu.getAngularOrientation();

        direction = 1;

        this.linearOpMode = linearOpMode;
        this.hardwareMap = hardwareMap;
    }
    public void initOpenCV(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
        //webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        //camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"));//, cameraMonitorViewId);
        pipeline = new barcodePipeline();
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                // Usually this is where you'll want to start streaming from the camera (see section 4)
            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }
    public void moveStraight(double inches, double speed, double direction){
        while ((double)Math.abs(leftFront.getCurrentPosition())/TICKS_TO_INCH_FORWARD <= inches){
            leftFront.setPower(direction * speed);
            leftBack.setPower(direction * speed);
            rightFront.setPower(direction * speed);
            rightBack.setPower(direction * speed);

            linearOpMode.telemetry.addData("leftFront:",leftFront.getCurrentPosition());
            linearOpMode.telemetry.update();
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    //if direction is 1, strafe right, if direction is -1 strafe left
    public void strafe(double inches, int direction, double speed){
        while (((double)(Math.abs(leftFront.getCurrentPosition()))/ TICKS_TO_INCH_STRAFE) <= inches){
            leftFront.setPower(direction * speed);
            leftBack.setPower(-direction * speed);
            rightFront.setPower(-direction * speed);
            rightBack.setPower(direction * speed);

            linearOpMode.telemetry.addData("rightFront:", rightFront.getCurrentPosition());
            linearOpMode.telemetry.update();
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // doesn't work scuffed angle unless we use sleep
    public void turnTo(double angle, double speed){
        int direction = (int)((imu.getAngularOrientation().firstAngle-angle)/Math.abs(imu.getAngularOrientation().firstAngle-angle));
        while (angleWrap(Math.abs(angle - imu.getAngularOrientation().firstAngle)) > 0.03){
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
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void spinCarousel(int direction){
        //-1 means spin left, 1 means spin right
        spin.setPower(direction * 0.38);
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

    //Moves the Arm and Wrist in sync at a given ratio depending on their position
    public void moveTwoMotors(double targetPositionArm, double targetPositionWrist, double inc){
        double startArm = arm.getPosition();
        double startWrist = wrist.getPosition();

        double loopRepititions = Math.abs(targetPositionArm - startArm)/inc;
        double wristInc = (targetPositionWrist - startWrist)/loopRepititions;

        for (int i= 0; i < loopRepititions; i++){
            arm.setPosition(startArm - (i * inc));
            wrist.setPosition(startWrist - (i * wristInc));
        }

        arm.setPosition(targetPositionArm);
        wrist.setPosition(targetPositionWrist);
    }

    // sets arm position to 3 (top layer of shipping hub)
//    public void armTo3(){
//        moveTwoMotors(0.5, 0.28, 0.02); //targetPositionArm was originally 0.65
////        for (int i = 0; i < 10; i++){
////            arm.setPosition(0.95 - (i *0.02));
////            wrist.setPosition(0.42 - (i * 0.02));
////        }
//        arm.setPosition(0.18);
//    }

    //sets arm position to 2 (middle layer of shipping hub)
    public void armTo2(){
        moveTwoMotors(0.5, 0.28, 0.02);  //change later (test values)
        arm.setPosition(0); //might change later (test values)
    }
    //sets arm position to 1 (bottom layer of shipping hub)
    public void armTo1(){
        moveTwoMotors(0.5, 0.28, 0.02); //might change later (test values)
        arm.setPosition(0.18); //might change later (test values)
    }
    //resets the arm to the bottom, using moveTwoMotors to flatten the wrist so that it fits
    public void armReset() throws InterruptedException {
//        moveTwoMotors(0.65, 0.28, 0.02);
        moveTwoMotors(0.86, 0.34, 0.02);
        Thread.sleep(1000);
        wrist.setPosition(0.42);
    }
    public void wristDrop2(){
        wrist.setPosition(1);
    }
    public void wristDrop1(){
        wrist.setPosition(0.28);
    }
    public void wristReset(){
        wrist.setPosition(0.42);
    }

}
