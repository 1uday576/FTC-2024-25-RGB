package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.pidfController.PIDFLift;

import java.util.function.DoubleSupplier;

public class LiftDrive extends CommandBase {

    private final PIDFLift liftSubsystem;
    private final DoubleSupplier multiplier;
    private final double factor = 20;
    public LiftDrive(PIDFLift subsystem, DoubleSupplier m) {
        liftSubsystem = subsystem;
        multiplier = m;

        addRequirements(subsystem);
    }

//    @Override
//    public void execute() {
//        liftSubsystem.move((int) (factor * multiplier));
//    }

    @Override
    public void execute(){
        liftSubsystem.tmpMove(-multiplier.getAsDouble());
    }
}
