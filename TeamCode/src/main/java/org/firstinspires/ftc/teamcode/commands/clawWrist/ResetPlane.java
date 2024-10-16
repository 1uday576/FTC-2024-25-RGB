package org.firstinspires.ftc.teamcode.commands.clawWrist;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Plane;

public class ResetPlane extends CommandBase {
    private Plane planSubsystem;
    public ResetPlane(Plane subsystem) {
        planSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute(){
        planSubsystem.reset();
    }
}
