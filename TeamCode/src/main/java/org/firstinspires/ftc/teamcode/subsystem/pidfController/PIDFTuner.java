package org.firstinspires.ftc.teamcode.subsystem.pidfController;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class PIDFTuner extends OpMode {
    //All public static variables are values we can dynamically change in FTCDashboard
    public static double p = 0, i= 0, d= 0;
    public static double f= 0;
    public static int target = 0;

    private DcMotorEx lift;
    private PIDController controller;
    private double ticks_in_degree;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //a controller provided by FTLib that runs PID calculations for us
        controller = new PIDController(p, i, d);
        //The lift needs to be reverse for it to travel upwards
        lift = hardwareMap.get(DcMotorEx.class, "lift1");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        ticks_in_degree = 2607 / 90.0;
    }

    @Override
    public void loop() {

        controller.setPID(p, i, d);
        int liftPos  = lift.getCurrentPosition();
        double pid = controller.calculate(liftPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;
        //setPosition is never used, rather we continuously recalculate and adjust
        lift.setPower(power);

        //Easier time reading these variables exact value without looking at the graph
        //These values can be shown on a graph
        telemetry.addData("pos ", liftPos);
        telemetry.addData("target ", target);
        telemetry.update();
    }
}