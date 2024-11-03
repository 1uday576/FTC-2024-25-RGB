package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.pidfController.PIDFArm;

public class ArmIn extends CommandBase {
    private final PIDFArm arm;

    public ArmIn(PIDFArm subsystem) {
        arm = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void execute(){
        arm.armIn();
    }
}
