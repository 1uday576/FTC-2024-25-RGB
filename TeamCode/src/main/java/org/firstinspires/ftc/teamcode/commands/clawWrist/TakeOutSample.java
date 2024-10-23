package org.firstinspires.ftc.teamcode.commands.clawWrist;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.ClawWrist;

public class TakeOutSample extends CommandBase {

    private final ClawWrist clawSubsystem;

    public TakeOutSample(ClawWrist subsystem){
        clawSubsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        clawSubsystem.takeOutSample();
    }
}
