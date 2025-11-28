package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.teamcode.Constants;


import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class DriveBase {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    public DriveBase(HardwareMap hardwareMap){
        frontLeftDrive = hardwareMap.get(DcMotor.class, Constants.FRONT_LEFT);
        frontRightDrive = hardwareMap.get(DcMotor.class, Constants.FRONT_RIGHT);
        backLeftDrive = hardwareMap.get(DcMotor.class, Constants.BACK_LEFT);
        backRightDrive = hardwareMap.get(DcMotor.class, Constants.BACK_RIGHT);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
    }

    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;



    public void drive(double axial , double lateral, double yaw) {
        double max;
        frontLeftPower = -axial + lateral + yaw;
        frontRightPower = -axial - lateral - yaw;
        backLeftPower = -axial - lateral + yaw;
        backRightPower = -axial + lateral - yaw;

        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;

        }
        sendSpeeds();

    }

    private void sendSpeeds(){
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);

    }

}

