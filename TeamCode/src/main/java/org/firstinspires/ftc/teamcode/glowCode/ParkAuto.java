package org.firstinspires.ftc.teamcode.glowCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

  @Autonomous
    public class ParkAuto extends LinearOpMode {
        private final HardwareMapping robot = new HardwareMapping();


        @Override
        public void runOpMode() throws InterruptedException {
            robot.init(hardwareMap);

            telemetry.addData("Status", "Initialized");
            telemetry.update();


            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            if (opModeIsActive()) {



           /* // We want to start the bot at x: 10, y: -8, heading: 90 degrees
            Pose2d startPose = new Pose2d(10, -8, Math.toRadians(90));

            drives.setPoseEstimate(startPose);

            Trajectory traj1 = drives.trajectoryBuilder(startPose)
                  .forward(50)
                  .build();
            Trajectory traj2 = drives.trajectoryBuilder(traj1.end())
                  .strafeLeft(50)
                  .build();
            Trajectory traj3 = drives.trajectoryBuilder(traj2.end())
                  .back(50)
                  .build();
            Trajectory traj4 = drives.trajectoryBuilder(traj3.end()\
                  .strafeRight(50)
                  .build();

            drives.followTrajectory(traj1);
            drives.followTrajectory(traj2);
            drives.followTrajectory(traj3);
            drives.followTrajectory(traj4);

            drives.turn(90);
            drives.turn(-90);

            */
                robot.driveAtDirection(45,2200,0.3);






            }
        }
    }

