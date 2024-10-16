package org.firstinspires.ftc.teamcode.commands.clawWrist;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystem.Plane;

public class PlaneGroup extends SequentialCommandGroup {

    public PlaneGroup(Plane subsytem){
        addCommands(
                new ReleasePlane(subsytem),
                new ResetPlane(subsytem)
        );
//TODO failed
        addRequirements(subsytem);
    }
}
