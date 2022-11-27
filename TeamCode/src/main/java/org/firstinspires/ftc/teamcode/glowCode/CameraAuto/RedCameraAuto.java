/*package org.firstinspires.ftc.teamcode.glowCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.Range;


@Autonomous (name = "Red Camera Auto", group = "Concept")

public class RedCameraAuto extends LinearOpMode {
    private final HardwareMapping robot = new HardwareMapping();
    webcamSample.freightPos pos;

    @Override
    public void runOpMode() throws InterruptedException {

        Lindsey Added line to make auto
       robot.init(hardwareMap);
        org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive drives = new org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive(hardwareMap);
        End

         Wait for the game to begin
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            // where the block is
            telemetry.addData("position",pos);
            telemetry.update();
            ElapsedTime runtime2 = new ElapsedTime();
            switch (pos) {

                case LEFT:
                    //move to carousel
                    robot.driveAtDirection(0, 100, .3);
                    robot.driveAtDirection(270, 2000, .3);
                    // move left about 6" to align with left stone
                    //pick up duck
                    //robot.clawArm.setPower(0.3);
                    //sleep(700);
                    //robot.driveAtDirection(0, 700, .3);
                    //robot.claw.setPosition(1);
                    //sleep(100);
                    //robot.clawArm.setPower(0.3);
                    //sleep(100);
                    //robot.claw.setPosition(0);
                    //pause to get block
                    /*runtime2.reset();
                    while (runtime2.milliseconds() < 1200) {
                        if (robot.touchSensor.getState() == true) {
                            telemetry.addData("Digital Touch", "Is Not Pressed");
                        } else {
                            telemetry.addData("Digital Touch", "Is Pressed");
                            robot.intakeRight.setPower(0);
                            robot.intakeLeft.setPower(0);
                        }
                    }
                    robot.intakeRight.setPower(0);
                    robot.intakeLeft.setPower(0);
                    //back up
                    robot.driveAtDirection(180, 800, .3);
                    //move past left bridge
                    robot.driveAtDirection(270, 2000, .3);
                    //drop off stone-reverse intake
                    sleep(1000);
                    robot.intakeRight.setPower(1);
                    robot.intakeLeft.setPower(-1);
                    sleep(2000);
                    //stop intake
                    robot.intakeRight.setPower(0);
                    robot.intakeLeft.setPower(0);
                    //go right until lined up with second stone
                    robot.driveAtDirection(90, 3200, .3);
                    //drive forward with intake on
                    robot.intakeRight.setPower(-1);
                    robot.intakeLeft.setPower(1);
                    robot.driveAtDirection(0, 1100, .3);
                    //pause to get block
                    runtime2.reset();
                    while (runtime2.milliseconds() < 1000) {
                        if (robot.touchSensor.getState() == true) {
                            telemetry.addData("Digital Touch", "Is Not Pressed");
                        } else {
                            telemetry.addData("Digital Touch", "Is Pressed");
                            robot.intakeRight.setPower(0);
                            robot.intakeLeft.setPower(0);
                        }
                    }
                    robot.intakeRight.setPower(0);
                    robot.intakeLeft.setPower(0);
                    //back up
                    robot.driveAtDirection(180, 900, .3);
                    //go left past bridge
                    robot.driveAtDirection(270, 3200, .3);
                    //block out
                    robot.intakeRight.setPower(1);
                    robot.intakeLeft.setPower(-1);
                    sleep(1000);
                    //park
                    robot.driveAtDirection(90, 600, .3);
                    break;

                case CENTER:

                    //move right to align with stone
                    robot.driveAtDirection(90, 100, .3);
                    //drive forward about 30" with intake on
                    robot.intakeRight.setPower(-1);
                    robot.intakeLeft.setPower(1);
                    robot.driveAtDirection(0, 2000, .3);
                    //pause to get block
                    runtime2.reset();
                    while (runtime2.milliseconds() < 1000) {
                        if (robot.touchSensor.getState() == true) {
                            telemetry.addData("Digital Touch", "Is Not Pressed");
                        } else {
                            telemetry.addData("Digital Touch", "Is Pressed");
                            robot.intakeRight.setPower(0);
                            robot.intakeLeft.setPower(0);
                        }
                    }
                    robot.intakeRight.setPower(0);
                    robot.intakeLeft.setPower(0);
                    //back up
                    robot.driveAtDirection(180, 700, .3);
                    //same, but longer by 8" than left
                    robot.driveAtDirection(270, 2500, .3);
                    //drop off stone-reverse intake
                    sleep(1000);
                    robot.intakeRight.setPower(1);
                    robot.intakeLeft.setPower(-1);
                    sleep(2000);
                    //stop intake
                    robot.intakeRight.setPower(0);
                    robot.intakeLeft.setPower(0);
                    //go right, touching side wall
                    robot.driveAtDirection(90, 3400, .3);
                    //drive forward with intake on
                    robot.intakeRight.setPower(-1);
                    robot.intakeLeft.setPower(1);
                    robot.driveAtDirection(0, 1100, .3);
                    //pause to get block
                    runtime2.reset();
                    while (runtime2.milliseconds() < 1000) {
                        if (robot.touchSensor.getState() == true) {
                            telemetry.addData("Digital Touch", "Is Not Pressed");
                        } else {
                            telemetry.addData("Digital Touch", "Is Pressed");
                            robot.intakeRight.setPower(0);
                            robot.intakeLeft.setPower(0);
                        }
                    }
                    robot.intakeRight.setPower(0);
                    robot.intakeLeft.setPower(0);
                    //back up
                    robot.driveAtDirection(180, 800, .3);
                    //same but longer by 8" than left
                    robot.driveAtDirection(270, 3500, .3);
                    //block out

                    robot.intakeRight.setPower(1);
                    robot.intakeLeft.setPower(-1);
                    sleep(1000);
                    //park
                    robot.driveAtDirection(90, 600, .3);
                    break;

                case RIGHT:
                    //move right about a foot
                    robot.driveAtDirection(90, 500, .3);
                    //drive forward about 30" with intake on
                    robot.intakeRight.setPower(-1);
                    robot.intakeLeft.setPower(1);
                    robot.driveAtDirection(0, 2200, .3);
                    //pause to get block
                    runtime2.reset();
                    while (runtime2.milliseconds() < 1000) {
                        if (robot.touchSensor.getState() == true) {
                            telemetry.addData("Digital Touch", "Is Not Pressed");
                        } else {
                            telemetry.addData("Digital Touch", "Is Pressed");
                            robot.intakeRight.setPower(0);
                            robot.intakeLeft.setPower(0);
                        }
                    }
                    robot.intakeRight.setPower(0);
                    robot.intakeLeft.setPower(0);
                    //back up
                    robot.driveAtDirection(180, 850, .3);
                    //same, but longer by 16" than left
                    robot.driveAtDirection(270, 2800, .3);
                    //drop off stone-reverse intake
                    sleep(1000);
                    robot.intakeRight.setPower(1);
                    robot.intakeLeft.setPower(-1);
                    sleep(2000);
                    //stop intake
                    robot.intakeRight.setPower(0);
                    robot.intakeLeft.setPower(0);

                     */
                    /*// go right until 1" from wall
                    robot.driveAtDirection(90, 3200, .3);
                    //go 45 degrees to the right
                    robot.driveAtDirection(45, 650, .3);
                    //drive forward with intake on
                    robot.intakeRight.setPower(-1);
                    robot.intakeLeft.setPower(1);
                    robot.driveAtDirection(0, 700, .3);
                    robot.turnRight(35, .2);
                    //pause to get block
                    runtime2.reset();
                    while (runtime2.milliseconds() < 1000) {
                        if (robot.touchSensor.getState() == true) {
                            telemetry.addData("Digital Touch", "Is Not Pressed");
                        } else {
                            telemetry.addData("Digital Touch", "Is Pressed");
                            robot.intakeRight.setPower(0);
                            robot.intakeLeft.setPower(0);
                        }
                    }
                        robot.intakeRight.setPower(0);
                        robot.intakeLeft.setPower(0);
                    //back up
                    robot.turnLeft(35, .3);
                    robot.driveAtDirection(180, 700, .3);
                    // rotate 45 degrees to the left
                    *//*robot.turnLeft(250, .3);*//*
                    // go left past bridge
                    robot.driveAtDirection(270, 3700, .3);
                    //block out
                    sleep(1000);
                    robot.intakeRight.setPower(1);
                    robot.intakeLeft.setPower(-1);
                    sleep(2000);*/
                    //go right, touching side wall
                    /*
                    robot.driveAtDirection(90, 3400, .3);
                    //drive forward with intake on
                    robot.intakeRight.setPower(-1);
                    robot.intakeLeft.setPower(1);
                    robot.driveAtDirection(0, 1100, .3);
                    //pause to get block
                    runtime2.reset();
                    while (runtime2.milliseconds() < 1000) {
                        if (robot.touchSensor.getState() == true) {
                            telemetry.addData("Digital Touch", "Is Not Pressed");
                        } else {
                            telemetry.addData("Digital Touch", "Is Pressed");
                            robot.intakeRight.setPower(0);
                            robot.intakeLeft.setPower(0);
                        }
                    }
                    robot.intakeRight.setPower(0);
                    robot.intakeLeft.setPower(0);
                    //back up
                    robot.driveAtDirection(180, 800, .3);
                    //same but longer by 8" than left
                    robot.driveAtDirection(270, 3500, .3);
                    //block out

                    robot.intakeRight.setPower(1);
                    robot.intakeLeft.setPower(-1);
                    sleep(1000);

                    //park
                    robot.driveAtDirection(90, 600, .3);
                    break;




            }

        }
    }
}*/



