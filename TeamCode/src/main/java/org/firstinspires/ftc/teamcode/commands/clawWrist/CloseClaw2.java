package org.firstinspires.ftc.teamcode.commands.clawWrist;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.ClawWrist;

public class CloseClaw2 extends CommandBase {

    private ClawWrist clawWristSubsystem;
    public CloseClaw2(ClawWrist subsystem) {
        clawWristSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute(){
        clawWristSubsystem.closeClaw2();
    }
}
