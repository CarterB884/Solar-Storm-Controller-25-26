package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanism.Intake;
import org.firstinspires.ftc.teamcode.Mechanism.Shooter;

@TeleOp
public class TeleWork extends OpMode {
    private DriveBase driveBase = null;
    private Shooter shooter = null;
    private Intake intake = null;
    public ElapsedTime runtime = new ElapsedTime();
    @Override
    public void init() {
        shooter = new Shooter(hardwareMap, runtime, telemetry);
        driveBase = new DriveBase(hardwareMap);
        intake = new Intake(hardwareMap);
    }
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        driveBase.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        //shooter-----------------------------------------------------------------------------
        if (gamepad1.right_bumper){
            shooter.shoot();
        } else if (gamepad1.left_bumper) {
            shooter.shootRev();
        } else {
            shooter.stop();
        }

        //intake------------------------------------------------------------------------------
        if (gamepad1.right_trigger > 0.5){
            intake.spinIn();
        }
        else {
            intake.spinStop();
        }
        // roundabout--------------------------------------------------------------------------
        if (gamepad1.dpad_up){
            shooter.roundUp();
        }
        else {
            shooter.roundStop();
        }

//        telemetry.addData("Status", "Run Time: " + runtime.toString());
//
        telemetry.update();
    }


}

