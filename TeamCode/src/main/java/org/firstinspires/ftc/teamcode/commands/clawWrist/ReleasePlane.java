package org.firstinspires.ftc.teamcode.commands.clawWrist;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.ClawWrist;
import org.firstinspires.ftc.teamcode.subsystem.Plane;

import java.util.Timer;

public class ReleasePlane extends CommandBase {
    private Plane planSubsystem;
    public ReleasePlane(Plane subsystem) {
        planSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute(){
        planSubsystem.release();
    }
}
