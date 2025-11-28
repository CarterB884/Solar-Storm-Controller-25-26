package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanism.Shooter;

@TeleOp
public class TeleTest extends OpMode {
    private DriveBase driveBase = null;
    private Shooter shooter = null;
    @Override
    public void init() {
        shooter = new Shooter(hardwareMap);
        driveBase = new DriveBase(hardwareMap);
    }

    @Override
    public void loop() {
        driveBase.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

//        telemetry.addData("Status", "Run Time: " + runtime.toString());
//
//        telemetry.update();
    }


}

