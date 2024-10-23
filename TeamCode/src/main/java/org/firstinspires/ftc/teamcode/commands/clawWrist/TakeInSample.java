package org.firstinspires.ftc.teamcode.commands.clawWrist;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.ClawWrist;

public class TakeInSample extends CommandBase {
    private final ClawWrist clawSubsystem;

    public TakeInSample(ClawWrist subsystem){
        clawSubsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        clawSubsystem.takeInSample();
    }
}
