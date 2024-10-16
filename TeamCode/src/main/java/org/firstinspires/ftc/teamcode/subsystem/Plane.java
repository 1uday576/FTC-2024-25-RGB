package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Plane extends SubsystemBase {

    Servo servo;

    public Plane(HardwareMap hardwareMap){
        servo = hardwareMap.get(Servo.class, "plane");
        servo.setDirection(Servo.Direction.REVERSE);
    }

    public void release(){
        servo.setPosition(0.25);
    }
    public void reset(){
        servo.setPosition(0);
    }
}
