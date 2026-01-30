package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class DriveBase {

    public DcMotor frontLeftDrive = null;
    public DcMotor backLeftDrive = null;
    public DcMotor frontRightDrive = null;
    public DcMotor backRightDrive = null;
    private GoBildaPinpointDriver goBildaPinpointDriver = null;

    public DriveBase(HardwareMap hardwareMap, GoBildaPinpointDriver goBildaPinpointDriver){
        frontLeftDrive = hardwareMap.get(DcMotor.class, Constants.FRONT_LEFT);
        frontRightDrive = hardwareMap.get(DcMotor.class, Constants.FRONT_RIGHT);
        backLeftDrive = hardwareMap.get(DcMotor.class, Constants.BACK_LEFT);
        backRightDrive = hardwareMap.get(DcMotor.class, Constants.BACK_RIGHT);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        this.goBildaPinpointDriver = goBildaPinpointDriver;
    }

    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;

    //auto rotate to april tag----------------------------------------------------------------------

//    public void autoRotate(double rotationPower) {
//    drive();
//    }


    // change---------------------------------------------------------------------------------------
    public void fieldRelativeDrive(Gamepad gamepad) {
        double y = -gamepad.left_stick_y;
        double x = gamepad.left_stick_x;
        double rx = gamepad.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad.options) {
            goBildaPinpointDriver.recalibrateIMU();
        }


        double botHeading = goBildaPinpointDriver.getHeading(UnnormalizedAngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        frontLeftPower = (rotY + rotX + rx) / denominator;
        backLeftPower = (rotY - rotX + rx) / denominator;
        frontRightPower = (rotY - rotX - rx) / denominator;
        backRightPower = (rotY + rotX - rx) / denominator;

        sendSpeeds();
    }
    public void autoRotate ( double rotationPower){
        frontLeftPower = rotationPower;
        frontRightPower = -rotationPower;
        backLeftPower = rotationPower;
        backRightPower = -rotationPower;
        sendSpeeds();
    }

    //------------------------------- --------------------------------------------------------------



    public void fieldRelativeDriveWithYaw(Gamepad gamepad, double yawAssist) {
        double y = -gamepad.left_stick_y;
        double x = gamepad.left_stick_x;
        double rx = gamepad.right_stick_x + yawAssist;

        if (gamepad.options) {
            goBildaPinpointDriver.recalibrateIMU();
        }

        double botHeading = goBildaPinpointDriver.getHeading(UnnormalizedAngleUnit.RADIANS);
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        frontLeftPower = (rotY + rotX + rx) / denominator;
        backLeftPower = (rotY - rotX + rx) / denominator;
        frontRightPower = (rotY - rotX - rx) / denominator;
        backRightPower = (rotY + rotX - rx) / denominator;

        sendSpeeds();


    }

    private void sendSpeeds(){
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);

    }

}

