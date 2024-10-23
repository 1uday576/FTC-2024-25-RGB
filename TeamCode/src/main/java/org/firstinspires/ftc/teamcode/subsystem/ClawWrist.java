package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawWrist extends SubsystemBase {
    private CRServo intake;
    public ClawWrist(HardwareMap hardwareMap) {

        intake = new CRServo(hardwareMap, "intake");
//        intake.setInverted(true);

    }

    public void takeInSample(){
        intake.set(0.8);
    }

    public void takeOutSample(){
        intake.set(-0.8);
    }

    public void stop(){
        intake.stop();
    }
}
