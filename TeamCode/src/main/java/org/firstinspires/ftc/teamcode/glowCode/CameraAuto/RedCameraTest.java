package org.firstinspires.ftc.teamcode.glowCode.CameraAuto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.glowCode.HardwareMapping;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class RedCameraTest extends LinearOpMode
{
    private final HardwareMapping robot = new HardwareMapping();

    //INTRODUCE VARIABLES HERE

    enum conePos {
        NULL, LEFT, MIDDLE, RIGHT
    }
    conePos pos = conePos.NULL;

    OpenCvCamera camera;
    EOAprilPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new EOAprilPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        Pose2d startPose = new Pose2d(-36, -62, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        /*Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .strafeRight(6)
                .build();

         */

        Trajectory traj2 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-40, -62), Math.toRadians(180))
                .addTemporalMarker(0.01, () -> {
                    robot.claw.setPosition(0.2);
                    robot.claw2.setPosition(0.2);
                })
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineTo(new Vector2d(-36, 10), Math.toRadians(0))
                .addTemporalMarker(0.01, () -> {
                    robot.moveToPositionArm(-100, 1);
                })
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .forward(6)
                .build();
        //drive.followTrajectoryAsync(traj3);
        telemetry.setMsTransmissionInterval(50);



        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT) {
                        pos = conePos.LEFT;
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                    if (tag.id == MIDDLE) {
                        pos = conePos.MIDDLE;
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                    if (tag.id == RIGHT) {
                        pos = conePos.RIGHT;
                        tagOfInterest = tag;
                        tagFound = true;
                        break;

                    }
                }


                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }
            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);

            robot.claw.setPosition(0.25);
            robot.claw2.setPosition(0.25);
        }


        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }


        //PUT AUTON CODE HERE (DRIVER PRESSED THE PLAY BUTTON!)
        telemetry.addData("position", pos);
        telemetry.update();
        switch (pos) {

            case LEFT:
                //drive.followTrajectory(traj1);
                drive.followTrajectory(traj2);
                drive.followTrajectory(traj3);
                robot.moveToPositionArm(-4300, 1);
                sleep(100);
                drive.followTrajectory(traj4);
                //robot.driveAtDirection(0, 200, 0.7);
                robot.claw.setPosition(0.5);
                robot.claw2.setPosition(0);
                break;

            case MIDDLE:
                //deliver preloaded cone to terminal
                //robot.driveAtDirection(270, 2000, .3);
                //move back to original position
                //robot.driveAtDirection(90, 2000, .3);
                //park in zone
                robot.driveAtDirection(0, 2000, .3);
                break;

            case RIGHT:
                //deliver preloaded cone to terminal
                //robot.driveAtDirection(270, 2000, .3);
                //move to "right" zone
                robot.driveAtDirection(90, 1800, .3);
                sleep(1000);
                //park in zone
                robot.driveAtDirection(0, 1500, .3);
                break;

            case NULL:
                robot.driveAtDirection(270, 2000, .3);
        }
    }
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        if (detection.id == 1) {
            telemetry.addLine(String.format("Left"));
        }
        else if (detection.id == 2){
            telemetry.addLine(String.format("Middle"));
        }
        else if (detection.id == 3){
            telemetry.addLine(String.format("Right"));
        }
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}