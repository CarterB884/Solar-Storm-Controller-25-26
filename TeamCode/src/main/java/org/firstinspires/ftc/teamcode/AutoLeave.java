package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Basic: AutoLeave", group="Linear OpMode")
//@Disabled
public class AutoLeave extends LinearOpMode {
    private DriveBase driveBase = new DriveBase();
    public void runOpMode() {
        driveBase.init(hardwareMap.get(DcMotor.class, "fl"),
                hardwareMap.get(DcMotor.class, "bl"),
                hardwareMap.get(DcMotor.class, "fr"),
                hardwareMap.get(DcMotor.class, "br"));
        while (!opModeIsActive());
        driveBase.takeInputs(0.5,0,0, false, false);
        driveBase.sendSpeeds();
        sleep(1000);
        driveBase.takeInputs(0,0,0, false, false);
        driveBase.sendSpeeds();

    }
};
