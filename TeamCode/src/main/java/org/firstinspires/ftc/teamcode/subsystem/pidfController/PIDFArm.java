package org.firstinspires.ftc.teamcode.subsystem.pidfController;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PIDFArm extends SubsystemBase {
    private PIDController controller;

    //The 'f' from the video = 'Kcos' from CTRL ALT FTC documentation
    private double p = 0, i = 0, d = 0, f = 0;

    private final double inch_per_tick = 1; //Total number of ticks in a degree

    public static int target = 0; //target position
    public static int currentRead = 0;
    private int error = -100;

    private final DcMotor arm;

    private final int OUT = 2060;
    private final int IN = 0;

    public static double lengthInch = 0.0;
    public static double angleRad = 0.0;
    public static int tickLimit = 0;

    public PIDFArm(HardwareMap hardwareMap, int tolerance) {
//        controller = new PIDController(p, i, d);
//        controller.setTolerance(tolerance);

        arm = hardwareMap.get(DcMotorEx.class, "arm");
//        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        //TODO these can be remove once PIDF is setup
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void move(int posChange){
        target += posChange;

        setPosition(target);
    }

    public void autonPos(int t){
        while(!controller.atSetPoint()){
            setPosition(t);
        }
    }

    public int limit(){
        angleRad = Math.toRadians(PIDFLift.currentRead / PIDFLift.ticks_in_degree);
        lengthInch = 20.0/Math.cos(angleRad);
        tickLimit = (int) Math.round(lengthInch * inch_per_tick);

        return tickLimit;
    }

    public void setPosition(int t){
        target = t;

        int armPos  = arm.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
//        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid;
        arm.setPower(power);
    }

    //temporary arm movement code
    public void tmpMove(double pos){

//        if(pos > 0){
//            int limit = limit();
//            if(target >= limit+error) target = limit;
//            else target += 5;
//            arm.setTargetPosition(target);
//            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            arm.setPower(1);
//        }else if(pos < 0){
//            if(!(target <= 50)){
//                target -= 5;
//                arm.setTargetPosition(target);
//                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                arm.setPower(1);
//            } else{
//                arm.setPower(0.0);
//            }
//        }
        //temporary calculation of limit
        limit();
        currentRead = arm.getCurrentPosition();
        PIDFLift.newDown = PIDFLift.lift1.getCurrentPosition();
        if(pos > 0){
            if(target >= 2060) return;
            target += (int)(30*pos);
            arm.setTargetPosition(target);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1);
        }else if(pos < 0){
            if(!(target <= 30)){
                target -= (int) (30*(-pos));
                arm.setTargetPosition(target);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(1);
            } else{
                arm.setTargetPosition(target);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(1);
            }
        }
    }

    public void armOut(){
        if (arm.getCurrentPosition() >= OUT) return;
        target = OUT;
        arm.setTargetPosition(OUT);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);
    }

    public void armIn(){
        if(arm.getCurrentPosition() <= IN ) return;
        target = IN;
        arm.setTargetPosition(IN);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);
    }

    public void tmpAutonPos(int t){
        while(arm.isBusy()){
            tmpSetPost(t);
        }
    }

    public void tmpSetPost(int t){
        target = t;
        arm.setTargetPosition(target);
        arm.setPower(0.25);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void tune(int target, double  p, double i, double d, double f){
        controller.setPID(p, i, d);
        int armPos  = arm.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
//        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid;
        arm.setPower(power);
    }

    public int armPos(){
        return arm.getCurrentPosition();
    }
}
