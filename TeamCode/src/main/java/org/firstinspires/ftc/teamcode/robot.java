package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.vision.barcodePipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import static org.firstinspires.ftc.teamcode.driveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.driveConstants.kA;
import static org.firstinspires.ftc.teamcode.driveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.driveConstants.kV;
import static org.firstinspires.ftc.teamcode.driveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.driveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.driveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.driveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.driveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.driveConstants.MOTOR_VELO_PID;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class robot extends MecanumDrive {

    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(9, 0, 0.4);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(7, 0, 0.3);

    public boolean RUN_USING_ENCODER;
    public static double LATERAL_MULTIPLIER = 60/43.2;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    DcMotorEx leftBack, leftFront, rightBack, rightFront;
    private List<DcMotorEx> motors;

    DcMotor spin, intake;
    Servo wrist, arm;
    private VoltageSensor batteryVoltageSensor;

    BNO055IMU imu;
    RevBlinkinLedDriver blinkinLedDriver;
    Orientation currentAngle;
    public static int barcode;

    LinearOpMode linearOpMode;
    HardwareMap hardwareMap;

    private TrajectoryFollower follower;

    public final int DIRECTION = 1;
    final private double CAROUSEL_SPEED = 0.46;
    final static double TICKS_TO_INCH_FORWARD = 37.87;
    final static double TICKS_TO_INCH_STRAFE = 70.68;
    static DcMotor[] encoderMotors;

    public robot(HardwareMap hardwareMap, LinearOpMode linearOpMode) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        //rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        //leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        encoderMotors = new DcMotorEx[]{leftFront, leftBack, rightFront, rightBack};

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
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
        motors = Arrays.asList(leftFront, leftBack, rightBack, rightFront);

        this.linearOpMode = linearOpMode;
        this.hardwareMap = hardwareMap;
        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()

        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));

        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);
    }

    public void moveStraight(double inches, double speed, double direction) {
        while (Util.getAverageEncoderPosition(encoderMotors) / TICKS_TO_INCH_FORWARD <= inches) {
            Util.setMotors(direction * speed, this);
            linearOpMode.telemetry.addData("leftFront:", leftFront.getCurrentPosition());
            linearOpMode.telemetry.update();
        }

        Util.setMotors(0, this);
        Util.resetAllEncoders(encoderMotors);
    }

    //if direction is 1, strafe right, if direction is -1 strafe left
    public void strafe(double inches, int direction, double speed) {
        while ((Util.getAverageEncoderPosition(encoderMotors) / TICKS_TO_INCH_STRAFE) <= inches) {
            //Util.setMotors(speed, direction, -direction, -direction, direction, this);
            leftBack.setPower(-direction * speed);
            leftFront.setPower(direction * speed);
            rightBack.setPower(direction * speed);
            rightFront.setPower(-direction * speed);

            linearOpMode.telemetry.addData("Average Encoders:", Util.getAverageEncoderPosition(encoderMotors));
            linearOpMode.telemetry.addData("lf", leftFront.getPower());
            linearOpMode.telemetry.addData("lb", leftBack.getPower());
            linearOpMode.telemetry.addData("rf", rightFront.getPower());
            linearOpMode.telemetry.addData("rb", rightBack.getPower());
            linearOpMode.telemetry.update();
        }

        Util.setMotors(0, this);
        Util.resetAllEncoders(encoderMotors);
    }

    // doesn't work scuffed angle unless we use sleep
    public void turnTo(double angle, double speed) {
        int direction = (int) ((imu.getAngularOrientation().firstAngle - angle) / Math.abs(imu.getAngularOrientation().firstAngle - angle));
        while (Util.angleWrap(Math.abs(angle - imu.getAngularOrientation().firstAngle)) > 0.05) {
            Util.setMotors(speed, direction, direction, -direction, -direction, this);
            //linearOpMode.telemetry.addData("angle", imu.getAngularOrientation().firstAngle);
            //linearOpMode.telemetry.addData("current-goal", Math.abs(imu.getAngularOrientation().firstAngle - angle));
            //linearOpMode.telemetry.update();
        }
        Util.setMotors(0, this);
        Util.resetAllEncoders(encoderMotors);
    }

    public void resetEncoders(){
//        for (int i = 0; i < encoderMotors.length; i++){
//            encoderMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            encoderMotors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        }
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    public void spinCarousel(int direction) {
        //-1 means spin left, 1 means spin right
        spin.setPower(direction * CAROUSEL_SPEED);
    }

    //Moves the Arm and Wrist in sync at a given ratio depending on their position
    public void moveTwoMotors(double targetPositionArm, double targetPositionWrist, double inc) {
        double startArm = arm.getPosition();
        double startWrist = wrist.getPosition();

        double loopRepititions = Math.abs(targetPositionArm - startArm) / inc;
        double wristInc = (targetPositionWrist - startWrist) / loopRepititions;

        for (int i = 0; i < loopRepititions; i++) {
            arm.setPosition(startArm - (i * inc));
            wrist.setPosition(startWrist - (i * wristInc));
        }

        arm.setPosition(targetPositionArm);
        wrist.setPosition(targetPositionWrist);
    }

    public void armToMiddle(){
        moveTwoMotors(0.5, 0.28, 0.02);
        arm.setPosition(0.35);
    }

    public void armToBottom() throws InterruptedException {
        moveTwoMotors(0.5, 0.24, 0.02);
        arm.setPosition(0.2);
        wrist.setPosition(0.18);//change later (test values)
//        Thread.sleep(2000);
//        arm.setPosition(0); //might change later (test values)
    }

    public void armToTop() {
        moveTwoMotors(0.5, 0.28, 0.02); //might change later (test values)
        //arm.setPosition(0.5); //might change later (test values)
    }

    public void armReset() throws InterruptedException{
//        moveTwoMotors(0.65, 0.28, 0.02);
        moveTwoMotors(1.0, 0.25, 0.02);
        Thread.sleep(1000);
        wrist.setPosition(0.38);
    }

    public void wristDrop() {
        wrist.setPosition(0.6);
    }

    public void wristReset() {
        wrist.setPosition(0.42);
    }

    public void initOpenCV(OpenCvWebcam webcam) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new barcodePipeline());
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        OpenCvWebcam finalWebcam = webcam;
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    finalWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                    /*
                     * This will be called if the camera could not be opened
                     */
                }
            });
        }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftBack.setPower(v1);
        rightBack.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        // TODO: This must be changed to match your configuration
        //                           | Z axis
        //                           |
        //     (Motor Port Side)     |   / X axis
        //                       ____|__/____
        //          Y axis     / *   | /    /|   (IO Side)
        //          _________ /______|/    //      I2C
        //                   /___________ //     Digital
        //                  |____________|/      Analog
        //
        //                 (Servo Port Side)
        //
        // The positive x axis points toward the USB port(s)
        //
        // Adjust the axis rotation rate as necessary
        // Rotate about the z axis is the default assuming your REV Hub/Control Hub is laying
        // flat on a surface

        // To work around an SDK bug, use -zRotationRate in place of xRotationRate
        // and -xRotationRate in place of zRotationRate (yRotationRate behaves as
        // expected). This bug does NOT affect orientation.
        //
        // See https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/251 for details.
        return (double) -imu.getAngularVelocity().xRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public void flash() throws InterruptedException {
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_FOREST_PALETTE);
        blinkinLedDriver.wait(1);
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_END_TO_END_BLEND);
    }
    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
}
