package org.firstinspires.ftc.teamcode.subsystem.pidfController;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class PIDFLift extends SubsystemBase {
    //set the PID controller to one lift and mirror hte result to the other
    private PIDController controller;

    //The 'f' from the video = 'Kcos' from CTRL ALT FTC documentation
    private double p = 0.0045, i = 0, d = 0.00015, f = 0.06;

    public int target = 0;
    public static int currentRead = -1;
    public boolean override = false;
    public static double ticks_in_degree = 2607 / 90.0;
    public int target1 = 0;

    private final DcMotor lift1;

    public static final int UP = 2000;
    public static final int DOWN = 10;

    public PIDFLift(HardwareMap hardwareMap, Telemetry tel, int tolerance) {
        controller = new PIDController(p,i,d);
        controller.setTolerance(tolerance);

        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lift1.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void pidfUP(){
        target = UP;
    }
    public void pidfDOWN(){
        target = DOWN;
    }
    public void move(int posChange) {
        target += posChange;

        setPosition(target);
    }

    public void setPosition(int t) {
        target = t;

        int liftPos = lift1.getCurrentPosition();
        double pid = controller.calculate(liftPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;
        lift1.setPower(power);
    }

    //TODO temporary lift moving code
    public void tmpMove(double pos){
        currentRead = lift1.getCurrentPosition();
        if(pos > 0){
            if(target <= 60 && override){
                target = currentRead;
                override = false;
            }
            target += 5;
            lift1.setTargetPosition(target);
            lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift1.setPower(0.8);
        }else if(pos < 0){
            if(target <= 60){
                target = 0;
                override = true;
                lift1.setPower(0.0);
                lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }else {
                target -= 5;
                lift1.setPower(0.8);
                lift1.setTargetPosition(target);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

        }


    }

    public void liftUp(){
        if (lift1.getCurrentPosition() >= UP) return;
        target = UP;
        lift1.setTargetPosition(UP);
        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift1.setPower(0.8);
    }

    public void liftDown(){
        if(lift1.getCurrentPosition() <= DOWN ) return;
        target = DOWN;
        lift1.setTargetPosition(DOWN);
        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift1.setPower(0.8);
    }

    public void tmpAutonPos(int t){
        while(lift1.isBusy()){
            tmpSetPost(t);
        }
    }

    public void tmpSetPost(int t){
        target1=t;
        lift1.setTargetPosition(target1);
        lift1.setPower(0.75);
        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void tune(int target, double p, double i, double d, double f) {
        controller.setPID(p, i, d);
        int liftPos = lift1.getCurrentPosition();
        double pid = controller.calculate(liftPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;
        lift1.setPower(power);
    }

    public int lift1Pos(){
        return lift1.getCurrentPosition();
    }

}
