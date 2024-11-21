package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.Drivebase;
import org.firstinspires.ftc.teamcode.subsystem.pidfController.PIDFArm;
import org.firstinspires.ftc.teamcode.subsystem.pidfController.PIDFLift;

import java.util.Timer;

@Autonomous
public class Park extends LinearOpMode {
    @Override
    public void runOpMode()  {
        Drivebase drivebase = new Drivebase(hardwareMap);
        PIDFArm slide = new PIDFArm(hardwareMap, 0);
        waitForStart();
        long finish = 0;
        long start = System.nanoTime();
            slide.armIn();

//            finish = System.nanoTime();
//            telemetry.addData("",finish - start);
//            telemetry.update();
//            while(finish - start <= 5) {
//                telemetry.addData("",finish - start);
//                telemetry.update();
//                finish = System.nanoTime();
//                drivebase.move(1, 0, 0);
//            }
        drivebase.move(0,-0.40,0);
        sleep(5000);
    }
}
