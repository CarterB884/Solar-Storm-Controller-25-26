package org.firstinspires.ftc.teamcode.Mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;


public class Shooter {
    private DcMotor shooter = null;
    private DcMotor roundabout = null;
    public ElapsedTime runtime = null;
    public Telemetry telemetry = null;
    public Shooter(HardwareMap hardwareMap, ElapsedTime runtime, Telemetry telemetry){
        shooter = hardwareMap.get(DcMotor.class, Constants.SHOOT);
        shooter.setDirection(DcMotor.Direction.FORWARD);
        roundabout = hardwareMap.get(DcMotor.class, Constants.ROUNDABOUT);
        roundabout.setDirection(DcMotor.Direction.REVERSE);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done

        this.runtime = runtime;
        this.telemetry = telemetry;
    }

    private int prevPos = 0;
    private double prevTime = 0;
    public void shoot() {
        shooter.setPower(1);
        int deltaPos = shooter.getCurrentPosition() - prevPos;
        prevPos = shooter.getCurrentPosition();
        double deltaTime = runtime.time() - prevTime;
        prevTime = runtime.time();
        double velo = deltaPos / deltaTime;
        telemetry.addData("shooter velocity", velo);

    }
    public void shootRev() {
        shooter.setPower(-1);
    }
    public void roundUp(){
        roundabout.setPower(1);
    }
    public void roundDown(){
        roundabout.setPower(-1);
    }
    public void roundStop(){
        roundabout.setPower(0);
    }
    public void stop() {
        shooter.setPower(0);
    }
    private double speedFromTagDist(double ty) {
        double distFactor = Math.cos((ty+30)*Math.PI/180);
        return distFactor * 0.6 + 0.3;
    }
}
