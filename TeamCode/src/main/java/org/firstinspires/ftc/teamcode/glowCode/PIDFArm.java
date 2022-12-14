package org.firstinspires.ftc.teamcode.glowCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp
public class PIDFArm extends OpMode {
    private final HardwareMapping robot = new HardwareMapping();
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    double Kg = 0;

    public static int target = 0;

    private final double TicksInDegree = 384.5 / 360;

    private DcMotor ArmMotor;


    @Override
    public void init() {
    controller = new PIDController(p, i, d);
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    ArmMotor = hardwareMap.get(DcMotor.class, "turretArm");
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int armPos = ArmMotor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        //double ff = Math.cos(Math.toRadians(target / TicksInDegree)) * f;
        // tune till the slide holds itself in place.

        // PID for general movement of the system, feedforward removes the disturbance
        // this improves the responsiveness of the PID controller
        double power = pid + Kg;


        ArmMotor.setPower(power);

        telemetry.addData("pos ", armPos);
        telemetry.addData("target ", target);
        telemetry.update();
    }
}
