package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawWrist extends SubsystemBase {
    private CRServo intake;
    public ClawWrist(HardwareMap hardwareMap) {

        intake = hardwareMap.get(CRServo.class, "intake");
//        intake.setInverted(true);

    }

    public void takeInSample(){
        intake.setPower(0.8);
    }

    public void takeOutSample(){
        intake.setPower(-0.8);
    }

    public void stop(){
        intake.setPower(0);
    }
}
