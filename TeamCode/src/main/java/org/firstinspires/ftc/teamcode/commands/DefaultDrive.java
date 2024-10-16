package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Drivebase;

import java.util.function.DoubleSupplier;

public class DefaultDrive extends CommandBase {

    private final Drivebase m_drive;
    private final DoubleSupplier strafeSpeed;
    private final DoubleSupplier forwardSpeed;
    private final DoubleSupplier turn;


    public DefaultDrive(Drivebase subsystem, DoubleSupplier s,
                        DoubleSupplier f, DoubleSupplier t) {
        m_drive = subsystem;
        strafeSpeed = s;
        forwardSpeed = f;
        turn = t;

        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        m_drive.move(strafeSpeed.getAsDouble(), forwardSpeed.getAsDouble(), -turn.getAsDouble());
    }
}
