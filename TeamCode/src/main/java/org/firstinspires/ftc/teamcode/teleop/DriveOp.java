package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
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
import org.firstinspires.ftc.teamcode.subsystem.pidfController.PIDFArm;
import org.firstinspires.ftc.teamcode.subsystem.pidfController.PIDFLift;

@TeleOp
public class DriveOp extends CommandOpMode {
    //TODO somehow implement Command System into the robot control for minimizing iteration-by-iteration robot logic
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


        //TODO add presets for arm positions and lift positions with new commands
        GamepadButton a = toolPad.getGamepadButton(GamepadKeys.Button.A);
        GamepadButton b = toolPad.getGamepadButton(GamepadKeys.Button.B);
        GamepadButton x = toolPad.getGamepadButton(GamepadKeys.Button.X);
        GamepadButton yT = toolPad.getGamepadButton(GamepadKeys.Button.Y);
        GamepadButton leftBumber = toolPad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER);
        GamepadButton rightBumber = toolPad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
        GamepadButton dpadTop = toolPad.getGamepadButton(GamepadKeys.Button.DPAD_UP);
        GamepadButton dpadDown = toolPad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN);
        GamepadButton dpadRight = toolPad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT);


        GamepadButton yD = drivePad.getGamepadButton(GamepadKeys.Button.Y);


        drivebase = new Drivebase(hardwareMap);
        driveCommand = new DefaultDrive(drivebase, () -> drivePad.getLeftX(),
                ()-> drivePad.getLeftY(), ()-> drivePad.getRightX());
//
//        //TODO change tolerance if needed
        armSubsystem = new PIDFArm(hardwareMap, 0);
        armCommand = new ArmDrive(armSubsystem, ()->toolPad.getRightY());

//        //TODO change tolerance if needed
        liftSubsystem = new PIDFLift(hardwareMap, 0);
        liftCommand = new LiftDrive(liftSubsystem, ()-> toolPad.getLeftY());
        liftUpCommand = new LiftUp(liftSubsystem);
        dpadTop.whenActive(liftUpCommand);
        liftDownCommand = new LiftDown(liftSubsystem);
        dpadDown.whenActive(liftDownCommand);

        clawSubsystem = new ClawWrist(hardwareMap);
        takeInIntakeCommand = new TakeInSample(clawSubsystem);
        takeOutIntakeCommand = new TakeOutSample(clawSubsystem);
        stopIntakeCommand = new StopIntake(clawSubsystem);

        a.whenHeld(takeInIntakeCommand).whenReleased(stopIntakeCommand);
        b.whenHeld(takeOutIntakeCommand).whenReleased(stopIntakeCommand);

        drivebase.setDefaultCommand(driveCommand);
        armSubsystem.setDefaultCommand(armCommand);
        liftSubsystem.setDefaultCommand(liftCommand);

        telemetry.addData("", toolPad.getRightY());
        telemetry.update();
    }
}
