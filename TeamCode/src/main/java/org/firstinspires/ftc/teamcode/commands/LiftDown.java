package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.pidfController.PIDFLift;

public class LiftDown extends CommandBase {
    private final PIDFLift lift;
    public LiftDown(PIDFLift subsystem){
        lift = subsystem;
    }

    @Override
    public void execute() {
        lift.liftDown();
    }
}
