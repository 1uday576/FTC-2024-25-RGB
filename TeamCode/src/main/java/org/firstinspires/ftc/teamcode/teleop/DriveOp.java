package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ArmDrive;
import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.commands.LiftDown;
import org.firstinspires.ftc.teamcode.commands.LiftDrive;
import org.firstinspires.ftc.teamcode.commands.LiftUp;
import org.firstinspires.ftc.teamcode.commands.clawWrist.AllUpWrist;
import org.firstinspires.ftc.teamcode.commands.clawWrist.CloseClaw1;
import org.firstinspires.ftc.teamcode.commands.clawWrist.CloseClaw2;
import org.firstinspires.ftc.teamcode.commands.clawWrist.OpenClaw1;
import org.firstinspires.ftc.teamcode.commands.clawWrist.OpenClaw2;
import org.firstinspires.ftc.teamcode.commands.clawWrist.TwistWrist;
import org.firstinspires.ftc.teamcode.commands.clawWrist.ReleasePlane;
import org.firstinspires.ftc.teamcode.commands.clawWrist.UntwistWrist;
import org.firstinspires.ftc.teamcode.subsystem.ClawWrist;
import org.firstinspires.ftc.teamcode.subsystem.Drivebase;
import org.firstinspires.ftc.teamcode.subsystem.Plane;
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

    private ClawWrist clawWristSubsystem;
    private TwistWrist twistWristCommand;
    private UntwistWrist untwistWristCommand;
    private OpenClaw1 openClaw1Command;
    private CloseClaw1 closeClaw1Command;
    private OpenClaw2 openClaw2Command;
    private CloseClaw2 closeClaw2Command;
    private AllUpWrist upWristCommand;

    private Plane planeSubsystem;
    private ReleasePlane releaseCommand;


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


        clawWristSubsystem = new ClawWrist(hardwareMap);
        openClaw1Command = new OpenClaw1(clawWristSubsystem);
        a.whenActive(openClaw1Command);
        closeClaw1Command = new CloseClaw1(clawWristSubsystem);
        b.whenActive(closeClaw1Command);

        openClaw2Command = new OpenClaw2(clawWristSubsystem);
        x.whenActive(openClaw2Command);
        closeClaw2Command = new CloseClaw2(clawWristSubsystem);
        yT.whenActive(closeClaw2Command);

        twistWristCommand = new TwistWrist(clawWristSubsystem);
        leftBumber.whenActive(twistWristCommand);
        untwistWristCommand = new UntwistWrist(clawWristSubsystem);
        rightBumber.whenActive(untwistWristCommand);
        upWristCommand = new AllUpWrist(clawWristSubsystem);
        dpadRight.whenActive(upWristCommand);

        planeSubsystem = new Plane(hardwareMap);
        releaseCommand = new ReleasePlane(planeSubsystem);
        yD.whenActive(releaseCommand); //TODO see if their is a way to reset the servo after shooting

        drivebase.setDefaultCommand(driveCommand);
        armSubsystem.setDefaultCommand(armCommand);
        liftSubsystem.setDefaultCommand(liftCommand);

        telemetry.addData("", toolPad.getRightY());
        telemetry.update();
    }
}
