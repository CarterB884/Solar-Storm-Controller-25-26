package org.firstinspires.ftc.teamcode.Mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class Intake {
    private DcMotor intake = null;
    public Intake (HardwareMap hardwareMap){
        intake = hardwareMap.get(DcMotor.class, Constants.INTAKE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void spinIn(){
        intake.setPower(1);
    }
    public void spinOut(){
        intake.setPower(-1);
    }
    public void spinStop(){
        intake.setPower(0);
    }
}
