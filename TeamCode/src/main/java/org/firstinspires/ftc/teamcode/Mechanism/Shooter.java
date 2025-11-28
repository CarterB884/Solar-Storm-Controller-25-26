package org.firstinspires.ftc.teamcode.Mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;


public class Shooter {
    private DcMotor shooter = null;
    public Shooter(HardwareMap hardwareMap){
        shooter = hardwareMap.get(DcMotor.class, Constants.SHOOT);
        shooter.setDirection(DcMotor.Direction.FORWARD);


    }

    public void shoot() {
        shooter.setPower(1);
    }
    public void stop() {
        shooter.setPower(0);
    }
    private double speedFromTagDist(double ty) {
        double distFactor = Math.cos((ty+30)*Math.PI/180);
        return distFactor * 0.6 + 0.3;
    }
}
