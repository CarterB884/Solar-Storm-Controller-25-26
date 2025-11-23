package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Basic: AutoLeave", group="Linear OpMode")
//@Disabled
public class AutoLeave extends LinearOpMode {
    private DriveBase driveBase = new DriveBase();
    public void runOpMode() {
        driveBase.init(hardwareMap.get(DcMotor.class, "fl"),
                hardwareMap.get(DcMotor.class, "bl"),
                hardwareMap.get(DcMotor.class, "fr"),
                hardwareMap.get(DcMotor.class, "br"),
                hardwareMap.get(DcMotor.class, "shoot"),
                hardwareMap.get(CRServo.class, "left_feeder"),
                hardwareMap.get(CRServo.class, "right_feeder"),
                hardwareMap.get(Limelight3A.class, "limelight"));
        while (!opModeIsActive());
        driveBase.takeInputs(1,0,0, false, false, false);
        driveBase.sendSpeeds();
        sleep(500);
        driveBase.takeInputs(0,0,0, true, false, false);
        driveBase.sendSpeeds();
//        while (Math.abs(driveBase.limelight.getLatestResult().getTx()) > 1 && driveBase.limelight.getLatestResult().isValid()) {
//            driveBase.takeInputs(0,0,-driveBase.limelight.getLatestResult().getTx()/5, true, false, false);
//            driveBase.sendSpeeds();
//        }
//        sleep(3000);
//        driveBase.takeInputs(0,0,0, true, false, true);
//        driveBase.sendSpeeds();
//        sleep(500);
//        driveBase.takeInputs(0,0,0, true, false, false);
//        driveBase.sendSpeeds();
//        sleep(3000);
//        driveBase.takeInputs(0,0,0, true, false, true);
//        driveBase.sendSpeeds();
//        sleep(500);
//        driveBase.takeInputs(0,0,0, true, false, false);
//        driveBase.sendSpeeds();
//        sleep(3000);
//        driveBase.takeInputs(0,0,0, true, false, true);
//        driveBase.sendSpeeds();
//        sleep(500);
//        driveBase.takeInputs(0,0,0,false,false,false);
//
//
    }
}
