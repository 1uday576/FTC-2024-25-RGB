package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawWrist extends SubsystemBase {
    private Servo[] servos = new Servo[3];
    private Servo claw1, claw2, wrist;
    /*
        defaulted already to false
        false is open or not twisted
        true is close or twisted
     */
    private boolean[] state = new boolean[3];
    public final double CLOSE = 0; //degrees to close the claw //TODO set the degrees
    public final double OPEN = 0.1;
    public final double TWIST = 0.5; //Required amount of turn from the wrist for pixel placement on board
    public final double ALLUP = 1;
    public final double UNTWIST = 0.25;

    public static final int CLAW1 = 0, CLAW2  = 1, WRIST  = 2;
    public ClawWrist(HardwareMap hardwareMap) {
//        Servo servo1 = hardwareMap.get(Servo.class, "claw1");
//        servo1.setDirection(Servo.Direction.REVERSE);
//        servos[0] = servo1;
//
//        Servo servo2 =hardwareMap.get(Servo.class, "claw2");
//        servo2.setDirection(Servo.Direction.REVERSE);
//        servos[1] = servo2;
//
//        Servo servo3 = hardwareMap.get(Servo.class, "wrist");
//        servo3.setDirection(Servo.Direction.REVERSE);
//        servos[2] = servo3;

        claw1 = hardwareMap.get(Servo.class, "claw1");
        claw1.setDirection(Servo.Direction.FORWARD);

        claw2 =hardwareMap.get(Servo.class, "claw2");
        claw2.setDirection(Servo.Direction.REVERSE);

        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.setDirection(Servo.Direction.REVERSE);

    }

//    public void openClose(int name){
//        Servo claw = servos[name];
////        if(state[name]){
////            claw.setPosition(OPEN);
////            state[name] = false;
////        }else {
////            claw.setPosition(CLOSE);
////            state[name] = true;
////        }
//    }

    public void openClaw1(){
        claw1.setPosition(OPEN);
    }
    public void openClaw2(){
        claw2.setPosition(OPEN);
    }
    public void closeClaw1(){
        claw1.setPosition(CLOSE);
    }
    public void closeClaw2(){
        claw2.setPosition(CLOSE);
    }
    public void twist(){
        wrist.setPosition(TWIST);
    }
    public void unTwist(){
        wrist.setPosition(UNTWIST);
    }
    public void up(){
        wrist.setPosition(ALLUP);
    }

//    public void twistWrist(){
//        Servo wrist = servos[WRIST];
//
//        if(state[WRIST]){
//            wrist.setPosition(TWIST);
//            state[WRIST] = false;
//        }else{
//            wrist.setPosition(UNTWIST);
//            state[WRIST] = true;
//        }
//    }
}
