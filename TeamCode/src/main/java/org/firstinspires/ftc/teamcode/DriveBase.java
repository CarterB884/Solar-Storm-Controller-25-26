package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DriveBase {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor shooter = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    private double csp = 0.0;
    private DcMotor intake = null;
    private Gamepad prevpad1 = gamepad1;
    private Gamepad prevpad2 = gamepad2;
    private Limelight3A limelight;
    public void init(DcMotor frontLeftDrive, DcMotor backLeftDrive, DcMotor frontRightDrive,DcMotor backRightDrive){
        this.frontLeftDrive = frontLeftDrive;
        this.backLeftDrive = backLeftDrive;
        this.frontRightDrive = frontRightDrive;
        this.backRightDrive = backRightDrive;
//        this.shooter = shooter;
//        shooter = hardwareMap.get(DcMotor.class, "shoot");
//        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
//        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
//        intake = hardwareMap.get(DcMotor.class, "in");
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");

//        telemetry.setMsTransmissionInterval(11);

//        limelight.pipelineSwitch(0);
//        limelight.start();

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
//        shooter.setDirection(DcMotor.Direction.FORWARD);
//        rightFeeder.setDirection(CRServo.Direction.REVERSE);
//        leftFeeder.setDirection(DcMotorSimple.Direction.FORWARD);

//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
    }
    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;
    double shooterPower;


    public void takeInputs(double axial , double lateral, double yaw, boolean shoot, boolean shootRev){
        double max;
        frontLeftPower  = axial + lateral + yaw;
        frontRightPower = axial - lateral - yaw;
        backLeftPower   = axial - lateral + yaw;
        backRightPower  = axial + lateral - yaw;

        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }
    }

    public void sendSpeeds(){
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
//        shooter.setPower(shooterPower);
//        leftFeeder.setPower(leftFeederPower);
//       rightFeeder.setPower(rightFeederPower);
    }

}

