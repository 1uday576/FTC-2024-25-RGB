package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ArmDrive;
import org.firstinspires.ftc.teamcode.subsystem.pidfController.PIDFArm;

@TeleOp
public class ArmTestOp extends CommandOpMode {
    private GamepadEx drivePad;
    private GamepadEx toolPad;
    private PIDFArm armSubsystem;
    private ArmDrive armCommand;

    @Override
    public void initialize() {
        drivePad = new GamepadEx(gamepad1);
        toolPad = new GamepadEx(gamepad2);

        armSubsystem = new PIDFArm(hardwareMap, 0);
        armCommand = new ArmDrive(armSubsystem, ()->toolPad.getRightY());

        armSubsystem.setDefaultCommand(armCommand);
    }
}
