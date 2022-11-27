package org.firstinspires.ftc.teamcode.glowCode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;


@TeleOp(name = "Test TeleOp")
public class TestTeleOp extends LinearOpMode {

    DcMotorEx ArmHeight;
    DcMotor Turret;
    CRServo ServoRight;
    CRServo ServoLeft;
    DcMotor RFMotor;
    DcMotor LFMotor;
    DcMotor RBMotor;
    DcMotor LBMotor;
    TouchSensor TS;

    double TicksCount1 = 2820;
    double TicksCount = 1850;

    @Override
    public void runOpMode() throws InterruptedException {

        ArmHeight = hardwareMap.get(DcMotorEx.class, "ArmHeight");
        Turret = hardwareMap.dcMotor.get("Turret");
        ServoRight = hardwareMap.crservo.get("ServoRight");
        ServoLeft = hardwareMap.crservo.get("ServoLeft");
        LFMotor = hardwareMap.dcMotor.get("LFMotor");
        RFMotor = hardwareMap.dcMotor.get("RFMotor");
        LBMotor = hardwareMap.dcMotor.get("LBMotor");
        RBMotor = hardwareMap.dcMotor.get("RBMotor");

        LBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LFMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        ArmHeight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmHeight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ArmHeight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {

            double powerMultiply;
            powerMultiply = 1 - gamepad1.right_trigger;
            double lateral = gamepad1.left_stick_x;
            double longitudinal = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double wheelPower = Math.hypot(lateral, longitudinal);
            double stickAngleRadians = Math.atan2(longitudinal, lateral);
            stickAngleRadians = stickAngleRadians - Math.PI / 4;
            double sinAngleRadians = Math.sin(stickAngleRadians);
            double cosAngleRadians = Math.cos(stickAngleRadians);
            double factor = 1 / Math.max(Math.abs(sinAngleRadians), Math.abs(cosAngleRadians));
            double LFPower = (wheelPower * cosAngleRadians * factor + turn) * powerMultiply;
            LFMotor.setPower(LFPower);
            double RFPower = (wheelPower * sinAngleRadians * factor - turn) * powerMultiply;
            RFMotor.setPower(RFPower);
            double LBPower = (wheelPower * sinAngleRadians * factor + turn) * powerMultiply;
            LBMotor.setPower(LBPower);
            double RBPower = (wheelPower * cosAngleRadians * factor - turn) * powerMultiply;
            RBMotor.setPower(RBPower);


            telemetry.addData("Current ArmHeight Position: ", ArmHeight.getCurrentPosition());
            telemetry.addData("Current Turret Position: ", Turret.getCurrentPosition());
            telemetry.addData("Curret Turret Power: ", Turret.getPower());
            telemetry.update();

            //Turret Movement

            if (gamepad2.dpad_up) {
                int targetPosition1 = 0;
                double CurrentPos1 = Turret.getCurrentPosition();
                if (Turret.getCurrentPosition() > targetPosition1) {
                    Turret.setTargetPosition(targetPosition1);
                    Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    double TDifference = Math.abs(Turret.getCurrentPosition() - Turret.getTargetPosition());
                    if (TDifference > 175) {
                        Turret.setPower(0.8);
                    } else if (TDifference < 175 && TDifference > 100) {
                        Turret.setPower(0.7);
                    } else if (TDifference < 100) {
                        Turret.setPower(0.6);
                    }
                } else if (Turret.getCurrentPosition() < targetPosition1) {
                    Turret.setTargetPosition( targetPosition1);
                    Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    double TDifference = Math.abs(Turret.getCurrentPosition() - Turret.getTargetPosition());
                    if (TDifference > 175) {
                        Turret.setPower(-0.6);
                    } else if (TDifference < 175 && TDifference > 100) {
                        Turret.setPower(-0.5);
                    } else if (TDifference < 100) {
                        Turret.setPower(-0.4);
                    }
                } else {
                    Turret.setPower(0);
                }
            }

            if (gamepad2.dpad_down) {
                double targetPosition1 = TicksCount1/2;
                double CurrentPos1 = Turret.getCurrentPosition();
                if (Turret.getCurrentPosition() < targetPosition1) {
                    Turret.setTargetPosition((int) targetPosition1);
                    Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    double TDifference = Math.abs(Turret.getCurrentPosition() - Turret.getTargetPosition());
                    if (TDifference > 175) {
                        Turret.setPower(0.8);
                    } else if (TDifference < 175 && TDifference > 100) {
                        Turret.setPower(0.7);
                    } else if (TDifference < 100) {
                        Turret.setPower(0.6);
                    }
                } else if (Turret.getCurrentPosition() > targetPosition1) {
                    Turret.setTargetPosition((int) targetPosition1);
                    Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    double TDifference = Math.abs(Turret.getCurrentPosition() - Turret.getTargetPosition());
                    if (TDifference > 175) {
                        Turret.setPower(-0.6);
                    } else if (TDifference < 175 && TDifference > 100) {
                        Turret.setPower(-0.5);
                    } else if (TDifference < 100) {
                        Turret.setPower(-0.4);
                    }
                } else {
                    Turret.setPower(0);
                }
            }

            if (gamepad2.dpad_left) {
                double targetPosition1 = TicksCount1 * 3/4;
                double CurrentPos1 = Turret.getCurrentPosition();
                if (Turret.getCurrentPosition() < targetPosition1) {
                    Turret.setTargetPosition((int) targetPosition1);
                    Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    double TDifference = Math.abs(Turret.getCurrentPosition() - Turret.getTargetPosition());
                    if (TDifference > 175) {
                        Turret.setPower(0.8);
                    } else if (TDifference < 175 && TDifference > 100) {
                        Turret.setPower(0.7);
                    } else if (TDifference < 100) {
                        Turret.setPower(0.6);
                    }
                } else if (Turret.getCurrentPosition() > targetPosition1) {
                    Turret.setTargetPosition((int) targetPosition1);
                    Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    double TDifference = Math.abs(Turret.getCurrentPosition() - Turret.getTargetPosition());
                    if (TDifference > 175) {
                        Turret.setPower(-0.6);
                    } else if (TDifference < 175 && TDifference > 100) {
                        Turret.setPower(-0.5);
                    } else if (TDifference < 100) {
                        Turret.setPower(-0.4);
                    }
                } else {
                    Turret.setPower(0);
                }
            }

            if (gamepad2.dpad_right) {
                double targetPosition1 = TicksCount1/4;
                double CurrentPos1 = Turret.getCurrentPosition();
                if (Turret.getCurrentPosition() < targetPosition1) {
                    Turret.setTargetPosition((int) targetPosition1);
                    Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    double TDifference = Math.abs(Turret.getCurrentPosition() - Turret.getTargetPosition());
                    if (TDifference > 175) {
                        Turret.setPower(0.8);
                    } else if (TDifference < 175 && TDifference > 100) {
                        Turret.setPower(0.7);
                    } else if (TDifference < 100) {
                        Turret.setPower(0.6);
                    }
                } else if (Turret.getCurrentPosition() > targetPosition1) {
                    Turret.setTargetPosition((int) targetPosition1);
                    Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    double TDifference = Math.abs(Turret.getCurrentPosition() - Turret.getTargetPosition());
                    if (TDifference > 175) {
                        Turret.setPower(-0.6);
                    } else if (TDifference < 175 && TDifference > 100) {
                        Turret.setPower(-0.5);
                    } else if (TDifference < 100) {
                        Turret.setPower(-0.4);
                    }
                } else {
                    Turret.setPower(0);
                }
            }



            if (gamepad2.y) {
                double targetPosition1 = TicksCount + 100;
                double CurrentPos1 = ArmHeight.getCurrentPosition();
                if (ArmHeight.getCurrentPosition() < targetPosition1) {
                    ArmHeight.setTargetPosition((int) targetPosition1);
                    ArmHeight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ArmHeight.setPower(0.75);
                } else if (ArmHeight.getCurrentPosition() > targetPosition1) {
                    ArmHeight.setTargetPosition((int) targetPosition1);
                    ArmHeight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ArmHeight.setPower(-0.75);
                } else {
                    ArmHeight.setPower(0);
                }
            } else if (gamepad2.a) {
                int targetPosition1 = 0;
                double CurrentPos1 = ArmHeight.getCurrentPosition();
                if (ArmHeight.getCurrentPosition() < targetPosition1) {
                    ArmHeight.setTargetPosition(targetPosition1);
                    ArmHeight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ArmHeight.setPower(0.75);
                } else if (ArmHeight.getCurrentPosition() > targetPosition1) {
                    ArmHeight.setTargetPosition((int) targetPosition1);
                    ArmHeight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ArmHeight.setPower(-0.75);
                } else {
                    ArmHeight.setPower(0);
                }
            }

            if (gamepad2.left_bumper) {
                ServoLeft.setPower(0.4);
                ServoRight.setPower(-0.4);
            } else if (gamepad2.right_bumper) {
                ServoLeft.setPower(-0.4);
                ServoRight.setPower(0.4);
            } else {
                ServoLeft.setPower(0);
                ServoRight.setPower(0);
            }
        }
    }
}

