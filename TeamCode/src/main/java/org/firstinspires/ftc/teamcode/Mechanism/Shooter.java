package org.firstinspires.ftc.teamcode.Mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SampleRevBlinkinLedDriver;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;
//
public class Shooter {
    private DcMotorEx shooter = null;
    private DcMotorEx shooter2 = null;
    private DcMotor roundabout = null;
//    private Servo aimLeft = null;
    public Servo aimRight = null;
    public ElapsedTime runtime = null;
    public Telemetry telemetry = null;
    private DistanceSensor ballSensor = null;
    private boolean ballInPosition = false;    // Ball ready?
    private boolean rpsReady = false;
    private ElapsedTime indexTimer = new ElapsedTime();
    private ElapsedTime rpsTimer = new ElapsedTime();
    private boolean indexing = false;
    public boolean shootCommanded = false;
    private boolean ballWasSeen = false;
    private ElapsedTime packTimer = new ElapsedTime();
    private double targetAimPos = 0.5;
    private static final double AIM_MIN = 0.35;  //testing parameters
    private static final double AIM_MAX = 0.65;



    public Shooter(HardwareMap hardwareMap, ElapsedTime runtime, Telemetry telemetry){
        shooter = hardwareMap.get(DcMotorEx.class, Constants.SHOOT);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter2 = hardwareMap.get(DcMotorEx.class, Constants.SHOOTER2);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        roundabout = hardwareMap.get(DcMotor.class, Constants.ROUNDABOUT);
        roundabout.setDirection(DcMotor.Direction.FORWARD);

        ballSensor = hardwareMap.get(DistanceSensor.class, Constants.BALL_SENSOR);
        indexTimer.reset();

//        aimLeft = hardwareMap.get(Servo.class, Constants.AIM_LEFT);
        aimRight = hardwareMap.get(Servo.class, Constants.AIM_RIGHT);
        aimRight.setPosition(0.5);
//        aimLeft.setDirection(Servo.Direction.FORWARD);
        aimRight.setDirection(Servo.Direction.REVERSE);
        //encoders----------------------------------------------------------------------------------
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        // Velocity PIDF setup
        shooter.setVelocityPIDFCoefficients(0.9, 0.2, 0.1, 12.8);
        shooter2.setVelocityPIDFCoefficients(0.9, 0.2, 0.1, 12.8);

        this.runtime = runtime;
        this.telemetry = telemetry;
    }

    private boolean revMode = false;
    private int prevPos = 0;
    private double prevTime = 0;
    public double targetRPS = 0;
    private static final double SERVO_SPEED = 0.005;
    private double currentServoPos = 0.4;
    public void setRevMode(boolean mode) { revMode = mode; }
    public void updateAimFromVision(double ty) {
        // Far preset = 0.5 (your current default)
        // Close shots only = slight UP adjustment (+0.05 max)
        double closeAdjust = 0;

        if (ty > 5.0) {  // Close to net (high ty)
            closeAdjust = 0.04;  // Slight up tilt only
        } else if (ty > 2.0) {
            closeAdjust = 0.02;  // Very slight
        }

        targetAimPos = 0.5 + closeAdjust;  // Far shots = no change
        targetAimPos = Math.max(AIM_MIN, Math.min(AIM_MAX, targetAimPos));
        aimRight.setPosition(targetAimPos);
    }

    public double getTargetAimPos() { return targetAimPos; }

    public void shoot() {
        targetRPS = 70;
    }
    public void shootslow(double distanceInches) {
       targetRPS = 60;
    }

//    0.675

    public void shootRev() {
        targetRPS = -50;
    }

//    public void shootDistance(double distanceInches) {
//        double rps;
//        if (distanceInches < 20) rps = 45;
//        else if (distanceInches < 40) rps = 55;
//        else rps = 65;
//        targetRPS = revMode ? -rps : rps;
//    }

    public void roundUp(){
        roundabout.setPower(0.6);
    }
    public void roundFah(){
        roundabout.setPower(1);
    }

    public void roundDown(){

        roundabout.setPower(-1);
    }

    public void roundStop(){
        roundabout.setPower(0);
    }

    public void stop() {
        targetRPS = 0;
    }

    public void setShootCommanded(boolean commanded) {
        shootCommanded = commanded;
    }




    public void updateVelocity() {
        shooter.setVelocity(targetRPS * 28);
        shooter2.setVelocity(targetRPS * 28);

        double sensorDistance = ballSensor.getDistance(DistanceUnit.INCH);
        ballInPosition = sensorDistance < Constants.BALL_PRESENT_DISTANCE;  // < 4"

        if (!rpsReady && shootCommanded && ballInPosition) {
            rpsTimer.reset();
        }

        rpsReady = (targetRPS > 0.5) && (Math.abs(shooter.getVelocity()) > (targetRPS * 28 * 0.98));

        // BOOST MODE: Not ready after 6 seconds
        boolean boostMode = (targetRPS > 0.5) && !rpsReady && (rpsTimer.time() > 6.0);
        if (boostMode) {
            targetAimPos = Math.min(currentServoPos + 0.08, AIM_MAX);
            aimRight.setPosition(targetAimPos);
        }

        if (rpsReady && shootCommanded && !indexing) {
            indexing = true;
            indexTimer.reset();
        }

        if (shootCommanded && indexing && indexTimer.time() > 0.25) {
            indexing = false;
        }

        if (!shootCommanded) {
            if (!ballInPosition) {
                roundUp();
            } else {
                roundStop();
            }
        } else {
            if (indexing) {
                roundFah();
            }
            else if (!ballInPosition) {
                roundUp();
            } else {
                roundStop();
            }
        }

        //TELEMETRY
        telemetry.addData("Target RPS", "%.1f", targetRPS);
        telemetry.addData("Sensor (in)", "%.1f", sensorDistance);
        telemetry.addData("Ball Ready", ballInPosition);
        telemetry.addData("RPS Ready", rpsReady);
        telemetry.addData("Boost Time", "%.1f s", rpsTimer.time());
//        telemetry.addData("Boost Mode", boostMode);
        telemetry.addData("AimPos", "%.3f", targetAimPos);
        telemetry.addData("Shoot Cmd", shootCommanded);
        telemetry.addData("Indexing", indexing);
    }



    public void aimUp() {
        currentServoPos = currentServoPos + SERVO_SPEED;
//        double currentPos = aimLeft.getPosition();
//        aimLeft.setPosition(newPos);
        aimRight.setPosition(currentServoPos);
    }

    public void aimDown() {
        currentServoPos =  currentServoPos - SERVO_SPEED;
//        double currentPos = aimLeft.getPosition();
//        double newPos = Math.max(0.15, currentPos - SERVO_SPEED);
//        aimLeft.setPosition(newPos);
        aimRight.setPosition(currentServoPos);
    }
    public void aimStop() {
//        aimLeft.setPosition(aimLeft.getPosition());  // Hold current position
//        aimRight.setPosition(aimRight.getPosition()); // Hold current position (brake)
    }



    private void safeServoCheck() {
//        double leftPos = aimLeft.getPosition();
        double rightPos = aimRight.getPosition();

//        if (leftPos < 0.1 || leftPos > 0.9 || rightPos < 0.1 || rightPos > 0.9) {
//            aimLeft.setPosition(0.5);
//            aimRight.setPosition(0.5);
//            telemetry.addData("SERVO", "EMERGENCY RESET!");
//        }
    }

    }

