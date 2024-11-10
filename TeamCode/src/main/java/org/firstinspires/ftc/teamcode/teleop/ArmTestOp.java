package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ArmDrive;
import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.commands.LiftDown;
import org.firstinspires.ftc.teamcode.commands.LiftDrive;
import org.firstinspires.ftc.teamcode.commands.LiftUp;
import org.firstinspires.ftc.teamcode.commands.clawWrist.StopIntake;
import org.firstinspires.ftc.teamcode.commands.clawWrist.TakeInSample;
import org.firstinspires.ftc.teamcode.commands.clawWrist.TakeOutSample;
import org.firstinspires.ftc.teamcode.subsystem.ClawWrist;
import org.firstinspires.ftc.teamcode.subsystem.Drivebase;
import org.firstinspires.ftc.teamcode.subsystem.RoboInfo;
import org.firstinspires.ftc.teamcode.subsystem.pidfController.PIDFArm;
import org.firstinspires.ftc.teamcode.subsystem.pidfController.PIDFLift;

@TeleOp
public class ArmTestOp extends CommandOpMode {
    private GamepadEx drivePad;
    private GamepadEx toolPad;

    private Drivebase drivebase;
    private DefaultDrive driveCommand;

    private PIDFArm armSubsystem;
    private ArmDrive armCommand;

    private PIDFLift liftSubsystem;
    private LiftDrive liftCommand;
    private LiftUp liftUpCommand;
    private LiftDown liftDownCommand;

    private ClawWrist clawSubsystem;
    private TakeInSample takeInIntakeCommand;
    private TakeOutSample takeOutIntakeCommand;
    private StopIntake stopIntakeCommand;

    @Override
    public void initialize() {
        drivePad = new GamepadEx(gamepad1);
        toolPad = new GamepadEx(gamepad2);

        GamepadButton a = toolPad.getGamepadButton(GamepadKeys.Button.A);
        GamepadButton b = toolPad.getGamepadButton(GamepadKeys.Button.B);
        GamepadButton x = toolPad.getGamepadButton(GamepadKeys.Button.X);
        GamepadButton yT = toolPad.getGamepadButton(GamepadKeys.Button.Y);
        GamepadButton leftBumber = toolPad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER);
        GamepadButton rightBumber = toolPad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
        GamepadButton dpadTop = toolPad.getGamepadButton(GamepadKeys.Button.DPAD_UP);
        GamepadButton dpadDown = toolPad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN);
        GamepadButton dpadRight = toolPad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT);
        GamepadButton dpadLeft = toolPad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT);

        RoboInfo roboInfo = new RoboInfo(telemetry);

        drivebase = new Drivebase(hardwareMap);
        driveCommand = new DefaultDrive(drivebase, () -> drivePad.getLeftX(),
                ()-> drivePad.getLeftY(), ()-> drivePad.getRightX());

        armSubsystem = new PIDFArm(hardwareMap, 0);
        armCommand = new ArmDrive(armSubsystem, ()->toolPad.getRightY());

        liftSubsystem = new PIDFLift(hardwareMap, telemetry, 0);
        liftCommand = new LiftDrive(liftSubsystem, ()-> toolPad.getLeftY());
        liftUpCommand = new LiftUp(liftSubsystem);
        dpadTop.whenPressed(new InstantCommand(liftSubsystem::liftUp, liftSubsystem));
//        dpadTop.whenPressed(new InstantCommand(liftSubsystem::pidfUP, liftSubsystem));
        liftDownCommand = new LiftDown(liftSubsystem);
        dpadDown.whenPressed(new InstantCommand(liftSubsystem::liftDown, liftSubsystem));
//        dpadDown.whenPressed(new InstantCommand(liftSubsystem::pidfDOWN, liftSubsystem));

        clawSubsystem = new ClawWrist(hardwareMap);
        takeInIntakeCommand = new TakeInSample(clawSubsystem);
        takeOutIntakeCommand = new TakeOutSample(clawSubsystem);
        stopIntakeCommand = new StopIntake(clawSubsystem);

        a.whenHeld(takeInIntakeCommand);
        a.whenReleased(stopIntakeCommand);
        b.whenHeld(takeOutIntakeCommand);
        b.whenReleased(stopIntakeCommand);

        drivebase.setDefaultCommand(driveCommand);
        armSubsystem.setDefaultCommand(armCommand);
        liftSubsystem.setDefaultCommand(liftCommand);
        schedule(new RunCommand(roboInfo::updateInfo, roboInfo));
    }
}
