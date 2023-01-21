package org.firstinspires.ftc.teamcode.glowCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

//    Robot wheel mapping:
//            X FRONT X
//          X           X
//        X  FL       FR  X
//                X
//               XXX
//                X
//        X  BL       BR  X
//          X           X
//            X       X
//        */
@TeleOp (name = "MecTeleOp")
//@Disabled
public class MecTeleOp extends OpMode {
    private final HardwareMapping robot = new HardwareMapping();
    private double SLOW = 0.75;
    double test = 0;
    double speed = 1;
    //private double slow = 1;



    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.turretArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {

        /*if (gamepad1.x) {
            if (slow == 1) {
                slow = .6;
            } else if (slow == .6) {
                slow = 1;
            }
        }
        else if (gamepad1.y) {
                slow = 1;
        }
        */

            // left stick controls direction
            // right stick X controls rotation
            double gamepad1LeftY = -gamepad1.left_stick_y;
            double gamepad1LeftX = gamepad1.left_stick_x;
            double gamepad1RightX = gamepad1.right_stick_x * SLOW;
            double gamepad2LeftY = gamepad2.left_stick_y;
            //double gamepad2LeftY = -gamepad1.left_stick_y * .7 * slow;
            /*if (gamepad2LeftY < -0.8) {

            }*/ //leave this be for now, not in use - Miles

            // holonomic formulas
            double FrontLeftPrep = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            double FrontRightPrep = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            double BackRightPrep = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
            double BackLeftPrep = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;

            // clip the right/left values so that the values never exceed +/- 1
            FrontRightPrep = Range.clip(FrontRightPrep, -1, 1);
            FrontLeftPrep = Range.clip(FrontLeftPrep, -1, 1);
            BackLeftPrep = Range.clip(BackLeftPrep, -1, 1);
            BackRightPrep = Range.clip(BackRightPrep, -1, 1);


            double FrontRight = Math.pow(FrontRightPrep, 3);
            double FrontLeft = Math.pow(FrontLeftPrep, 3);
            double BackLeft = Math.pow(BackLeftPrep, 3);
            double BackRight = Math.pow(BackRightPrep, 3);

            // write the values to the motors
            robot.rightFront.setPower(FrontRight);
            robot.leftFront.setPower(FrontLeft);
            robot.leftRear.setPower(BackLeft);
            robot.rightRear.setPower(BackRight);

            robot.turretArm.setPower(gamepad2LeftY);
            //robot.clawArm.setPower(gamepad2.left_stick_x / 3);
            //robot.claw.setPosition(gamepad2.right_stick_x);
            //robot.claw2.setPosition(gamepad2.right_stick_x);

            //open
            if (gamepad2.right_bumper) {
               robot.claw.setPosition(0.5);
               robot.claw2.setPosition(0);
            }
            //close
            if (gamepad2.left_bumper) {
                robot.claw.setPosition(0.3);
                robot.claw2.setPosition(0.2);
            }
            if (gamepad2.y) {
                robot.moveToPositionArm(-460, 1, true);
            }
            if (gamepad2.a) {
                robot.moveToPositionArm(-4300, 1, true);
                //robot.turretArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            /*if (gamepad2.x) {
                robot.turretArm.setTargetPosition(Math.abs(robot.turretArm.getCurrentPosition())-robot.turretArm.getCurrentPosition());
                robot.turretArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.turretArm.setPower(speed);
                while (robot.turretArm.isBusy()) {

                }
            }

             */

            if (gamepad2.x) {
                test = test + 1;
                if (test == 1) {
                    robot.moveToPositionArm(-400, 1, true);
                }
                if (test == 2) {
                    robot.moveToPositionArm(100, 1, true);
                }
                if (test == 3) {
                    robot.moveToPositionArm(100, 1, true);
                }
                if (test == 4) {
                    robot.moveToPositionArm(100, 1, true);
                }
                if (test == 5) {
                    robot.moveToPositionArm(100, 1, true);
                }
            }



            //robot.claw.setPosition(gamepad2.right_trigger);
            /* Telemetry for debugging  */
            //telemetry.addData("Front Right:", robot.rightFront.isMotorEnabled());
            //telemetry.addData("Rmotor", robot.rightFront.getVelocity());
            //telemetry.addData("Lmotor", robot.leftFront.getVelocity());
            telemetry.addData("lift encoders", String.format("%d", robot.turretArm.getCurrentPosition()));
            telemetry.addData("Joy XL YL XR", String.format("%.2f", gamepad1LeftX) + " " +
                   String.format("%.2f", gamepad1LeftY) + " " + String.format("%.2f", gamepad1RightX));
            //telemetry.addData("Stick", gamepad1.right_stick_y);
            telemetry.addData("Encoders:", String.format("%d",robot.leftFront.getCurrentPosition()) + " " +
                            String.format("%d",robot.rightFront.getCurrentPosition()) + " " +
                            String.format("%d",robot.leftRear.getCurrentPosition()) + " " +
                            String.format("%d",robot.rightRear.getCurrentPosition()) + " " +
                            String.format("%d",robot.turretArm.getCurrentPosition()));
                    telemetry.update();

    }
}

