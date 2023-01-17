package org.firstinspires.ftc.teamcode.glowCode;

import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.AxisDirection;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;

import org.firstinspires.ftc.teamcode.util.Encoder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.Arrays;
import java.util.List;


public class HardwareMapping {

    /* Public OpMode members. */
    public DcMotorEx turretArm = null;
    public Servo claw = null;
    public Servo claw2 = null;

    public Encoder parallelEncoder = null;
    public Encoder perpendicularEncoder = null;

    /* local OpMode members. */
    public HardwareMap hwMap = null;
    private final ElapsedTime period = new ElapsedTime();

/* Copied out of SampleMecanumDrive*/
    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    public List<DcMotorEx> motors;
    public BNO055IMU imu;
/*End Copied out of SampleMecanumDrive*/

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

/*Copied out of SampleMecanumDrive*/
        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        //BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);
        BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_X);
        BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_Y);


        leftFront = hwMap.get(DcMotorEx.class, "leftFront");
        leftRear = hwMap.get(DcMotorEx.class, "leftRear");
        rightRear = hwMap.get(DcMotorEx.class, "rightRear");
        rightFront = hwMap.get(DcMotorEx.class, "rightFront");
        turretArm = hwMap.get(DcMotorEx.class, "turretArm");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

/*End Copied out of SampleMecanumDrive*/

        // Define and Initialize Motors
        //clawArm = hwMap.get(DcMotorEx.class, "clawArm");
        //clawArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Define and initialize ALL installed servos.
        claw = hwMap.get(Servo.class, "claw");
        claw2 = hwMap.get(Servo.class, "claw2");


        // set the digital channel to input.
        //claw.setPosition(0.5);
        //claw2.setPosition(0.3);

        WebcamName webcamName = hwMap.get(WebcamName.class, "Webcam 1");


    }

    public void moveToPosition(double driveDistance, double speed) {
         int move = (int) driveDistance;

         leftRear.setTargetPosition(leftRear.getCurrentPosition()+move);
         rightRear.setTargetPosition(rightRear.getCurrentPosition()+move);
         leftFront.setTargetPosition(leftFront.getCurrentPosition()+move);
         rightFront.setTargetPosition(rightFront.getCurrentPosition()+move);
         rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

          leftRear.setPower(speed);
                    rightRear.setPower(speed);
                              leftFront.setPower(speed);
                                        rightFront.setPower(speed);
           while (leftRear.isBusy()&&leftFront.isBusy()&&rightFront.isBusy()&&rightRear.isBusy()) {

           }
           leftRear.setPower(0);
                     rightRear.setPower(0);
                               rightFront.setPower(0);
                                        leftFront.setPower(0);

    }
    public void moveToPositionArm(int driveDistance, double speed) {
        turretArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretArm.setTargetPosition(Math.abs(turretArm.getCurrentPosition())+driveDistance);
        turretArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretArm.setPower(speed);
        while (turretArm.isBusy()) {

        }
        //turretArm.setPower(0);
        }
    /*public void armMove(int move, double speed) {
        turretArm.setTargetPosition(move);
        turretArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretArm.setPower(speed);

        //turretArm.setPower(0);
    }

     */


    public void clawPosition(double clawPos, double claw2Pos) {
       claw.setPosition(clawPos);
       claw2.setPosition(claw2Pos);

    }




    public void driveAtDirection(double AngleIn, double driveDistance, double motorPower) {
        double LeftYMotorFix = -1;
        double LeftXMotorFix = -1;
        double RightXMotorFix = -1;
        int move = (int) driveDistance;

        if (AngleIn == 0 || AngleIn == 360) {
            leftFront.setDirection(DcMotor.Direction.REVERSE);
            leftRear.setDirection(DcMotor.Direction.REVERSE);
            rightRear.setDirection(DcMotor.Direction.FORWARD);
            rightFront.setDirection(DcMotor.Direction.FORWARD);
        }

        if (AngleIn == 90) {
            rightFront.setDirection(DcMotor.Direction.REVERSE);
            leftRear.setDirection(DcMotor.Direction.FORWARD);
            rightRear.setDirection(DcMotor.Direction.FORWARD);
            leftFront.setDirection(DcMotor.Direction.REVERSE);
        }
        if (AngleIn == 180) {
            rightFront.setDirection(DcMotor.Direction.REVERSE);
            leftRear.setDirection(DcMotor.Direction.FORWARD);
            rightRear.setDirection(DcMotor.Direction.REVERSE);
            leftFront.setDirection(DcMotor.Direction.REVERSE);
        }
        if (AngleIn == 270) {
            leftFront.setDirection(DcMotor.Direction.REVERSE);
            leftRear.setDirection(DcMotor.Direction.REVERSE);
            rightRear.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setDirection(DcMotor.Direction.FORWARD);
        }


        leftRear.setTargetPosition(leftRear.getCurrentPosition()+move);
        rightRear.setTargetPosition(rightRear.getCurrentPosition()+move);
        leftFront.setTargetPosition(leftFront.getCurrentPosition()+move);
        rightFront.setTargetPosition(rightFront.getCurrentPosition()+move);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double frontLeftStart = leftRear.getCurrentPosition();
        double frontRightStart = rightRear.getCurrentPosition();

        double startingHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
        //double startingHeading = 0;

        while (Math.sqrt(Math.pow((leftRear.getCurrentPosition() - frontLeftStart),2)+Math.pow((rightRear.getCurrentPosition()-frontRightStart),2)) < driveDistance) {

            double LeftY;
            double LeftX;
            double RightX = 0;

            LeftX = Math.sin(Math.toRadians(AngleIn))*LeftXMotorFix*motorPower;
            LeftY = Math.cos(Math.toRadians(AngleIn))*LeftYMotorFix*motorPower;

            double currentHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;

            double correctionPower = 0;
            if (Math.abs(currentHeading-startingHeading)<.2){
//                correctionPower = 0;
                correctionPower = 0;
            }else if (Math.abs(currentHeading-startingHeading)<1) {
//                correctionPower = 0.005;
                correctionPower = motorPower*.15;
            }else if (Math.abs(currentHeading-startingHeading)<2.5) {
//                correctionPower = 0.01;
                correctionPower = motorPower*.2;
            }else if (Math.abs(currentHeading-startingHeading)<5) {
//                correctionPower = 0.015;
                correctionPower = motorPower*.25;
            }else if (Math.abs(currentHeading-startingHeading)<10) {
//                correctionPower = 0.02;
                correctionPower = motorPower*.3;
            }else if (Math.abs(currentHeading-startingHeading)<15) {
//                correctionPower = 0.03;
                correctionPower = motorPower*.35;
            }else if (Math.abs(currentHeading-startingHeading)<20) {
//                correctionPower = 0.04;
                correctionPower = motorPower*.4;
            }else if (Math.abs(currentHeading-startingHeading)>=20) {
//                correctionPower = 0.05;
                correctionPower = motorPower*.45;
            }


            if (startingHeading > currentHeading){
                RightX = -correctionPower*RightXMotorFix;
            }else if (startingHeading < currentHeading){
                RightX = correctionPower*RightXMotorFix;
            }else if (startingHeading == currentHeading){
                RightX = 0;
            }


            double FrontLeftPrep = -LeftY - LeftX - RightX;
            double FrontRightPrep = LeftY - LeftX - RightX;
            double BackRightPrep = LeftY + LeftX - RightX;
            double BackLeftPrep = -LeftY + LeftX - RightX;

            // clip the right/left values so that the values never exceed +/- 1
            double FrontRight = Range.clip(FrontRightPrep, -1, 1);
            double FrontLeft = Range.clip(FrontLeftPrep, -1, 1);
            double BackLeft = Range.clip(BackLeftPrep, -1, 1);
            double BackRight = Range.clip(BackRightPrep, -1, 1);

            // write the values to the motors
            rightFront.setPower(FrontRight);
            leftFront.setPower(FrontLeft);
            leftRear.setPower(BackLeft);
            rightRear.setPower(BackRight);
        }

        rightFront.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }
    public void turnRight(double driveDistanceRight, double motorPowerRight) {
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double frontLeftStart = leftRear.getCurrentPosition();

        while ((leftRear.getCurrentPosition() - frontLeftStart) < driveDistanceRight) {
            double RightX = motorPowerRight;

            // write the values to the motors
            rightFront.setPower(RightX);
            leftFront.setPower(RightX);
            leftRear.setPower(RightX);
            rightRear.setPower(RightX);
        }
    }

    public void turnLeft(double driveDistanceLeft, double motorPowerLeft) {
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double frontLeftStart = leftRear.getCurrentPosition();

        while (Math.abs(leftRear.getCurrentPosition() - frontLeftStart) < driveDistanceLeft) {
            double RightX = -motorPowerLeft;

            // write the values to the motors
            rightFront.setPower(RightX);
            leftFront.setPower(RightX);
            leftRear.setPower(RightX);
            rightRear.setPower(RightX);
        }
    }


}
