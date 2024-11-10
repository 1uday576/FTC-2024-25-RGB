package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TestOp extends LinearOpMode {

    public static double ticks_in_degree;
    @Override

    public void runOpMode() throws InterruptedException {
        DcMotor arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotor lift = hardwareMap.get(DcMotorEx.class, "lift1");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        ticks_in_degree = 2607.0 / 90.0;

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Arm position: ",arm.getCurrentPosition());
            telemetry.addData("Lift position: ", lift.getCurrentPosition());
            telemetry.addData("Lift Angle: ", lift.getCurrentPosition()/ticks_in_degree);
            telemetry.addData("Lift cos ratio: ",Math.cos(Math.toRadians(lift.getCurrentPosition() / ticks_in_degree)));
            telemetry.update();
        }
    }
}
