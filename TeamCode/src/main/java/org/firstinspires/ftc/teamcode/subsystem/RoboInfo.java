package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.pidfController.PIDFArm;
import org.firstinspires.ftc.teamcode.subsystem.pidfController.PIDFLift;

public class RoboInfo extends SubsystemBase {
    public Telemetry telemetry;
    public RoboInfo(Telemetry tel){
        telemetry = tel;
    }

    public void updateInfo(){
        telemetry.addLine("Lift Information:");
        telemetry.addData("Lit position: ", PIDFLift.currentRead);

        telemetry.addLine("Arm Information:");
        telemetry.addData("Arm current position: ", PIDFArm.currentRead);
        telemetry.addData("Arm target position: ", PIDFArm.target);
        telemetry.addData("Arm angle radians: ", PIDFArm.angleRad);
        telemetry.addData("Arm max Length (inch): ", PIDFArm.lengthInch);
        telemetry.addData("Arm max length (ticks): ", PIDFArm.tickLimit);
        telemetry.update();
    }
}
