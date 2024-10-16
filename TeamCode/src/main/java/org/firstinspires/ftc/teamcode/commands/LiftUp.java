package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.pidfController.PIDFLift;

import java.util.function.DoubleSupplier;

public class LiftUp extends CommandBase {
    private final PIDFLift lift;

    public LiftUp(PIDFLift subsystem){
        lift = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        lift.liftUp();
    }
}
