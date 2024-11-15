package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivebase extends SubsystemBase {
    private Motor frontLeft, frontRight, backLeft, backRight;

    private MecanumDrive drive;

    private final String FL = "frontLeftMotor", FR = "frontRightMotor";

    private final String BL = "backLeftMotor", BR = "backRightMotor";

    private final double DEAD_ZONE = 0.2;

    public Drivebase(HardwareMap hardwareMap){
        //By default the motors are in RawPower mode
        frontLeft = new Motor(hardwareMap, FL, Motor.GoBILDA.RPM_312);
        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontLeft.setInverted(true);

        frontRight = new Motor(hardwareMap, FR, Motor.GoBILDA.RPM_312);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setInverted(true);

        backLeft = new Motor(hardwareMap, BL, Motor.GoBILDA.RPM_312);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setInverted(true);

        backRight = new Motor(hardwareMap, BR, Motor.GoBILDA.RPM_312);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setInverted(true);

        drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

    }

    public void move(double leftX, double leftY, double rightX){
        double strafeSpeed = leftX > DEAD_ZONE || leftX < -DEAD_ZONE
                ? leftX
                : 0;

        double forwardSpeed = leftY > DEAD_ZONE || leftY < -DEAD_ZONE
                ? leftY
                : 0;

        double turn = rightX > DEAD_ZONE || rightX < -DEAD_ZONE
                ? rightX
                : 0;

        drive.driveRobotCentric(
                strafeSpeed,
                forwardSpeed,
                -turn
        );
    }
}
