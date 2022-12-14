package org.firstinspires.ftc.teamcode.glowCode.ArmCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.glowCode.HardwareMapping;

@Config
@TeleOp
public class PIDFArm extends OpMode {
    private final HardwareMapping robot = new HardwareMapping();
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    double Kg = 0;

    public static int target = 0;

    private final double TicksInDegree = 700 / 180.0;

    DcMotorEx turretArm;


    @Override
    public void init() {
    controller = new PIDController(p, i, d);
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    turretArm = hardwareMap.get(DcMotorEx.class, "turretArm");
    turretArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int armPos = turretArm.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / TicksInDegree)) * f;

        // tune till the slide holds itself in place.

        // PID for general movement of the system, feedforward removes the disturbance
        // this improves the responsiveness of the PID controller
        double power = pid + ff;


        turretArm.setPower(power);

        telemetry.addData("pos ", armPos);
        telemetry.addData("target ", target);
        telemetry.update();
    }
}
