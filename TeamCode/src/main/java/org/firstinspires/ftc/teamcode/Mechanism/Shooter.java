package org.firstinspires.ftc.teamcode.Mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Shooter {
    private DcMotorEx shooter = null;
    private DcMotorEx shooter2 = null;
    private DcMotor roundabout = null;
    private Servo aimLeft = null;
    private Servo aimRight = null;
    public ElapsedTime runtime = null;
    public Telemetry telemetry = null;
    private DistanceSensor ballSensor = null;
    private boolean ballInPosition = false;    // Ball ready?
    private boolean rpsReady = false;         // Shooter speed ready?
    private ElapsedTime indexTimer = new ElapsedTime();
    private boolean indexing = false;



    public Shooter(HardwareMap hardwareMap, ElapsedTime runtime, Telemetry telemetry){
        shooter = hardwareMap.get(DcMotorEx.class, Constants.SHOOT);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter2 = hardwareMap.get(DcMotorEx.class, Constants.SHOOTER2);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        roundabout = hardwareMap.get(DcMotor.class, Constants.ROUNDABOUT);
        roundabout.setDirection(DcMotor.Direction.FORWARD);

        ballSensor = hardwareMap.get(DistanceSensor.class, Constants.BALL_SENSOR);
        indexTimer.reset();

        aimLeft = hardwareMap.get(Servo.class, Constants.AIM_LEFT);
        aimRight = hardwareMap.get(Servo.class, Constants.AIM_RIGHT);
        aimLeft.setDirection(Servo.Direction.FORWARD);
        aimRight.setDirection(Servo.Direction.REVERSE);
        //encoders----------------------------------------------------------------------------------
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // Velocity PID mode
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // Velocity PID mode



        // Velocity PIDF setup (for fast RPM recovery...apparetly)
        PIDFCoefficients pidf = new PIDFCoefficients(0.8, 0.0, 0.1, 12.8);
        shooter.setVelocityPIDFCoefficients(0.8, 0.0, 0.1, 12.8);
        shooter2.setVelocityPIDFCoefficients(0.8, 0.0, 0.1, 12.8);
//for coaches(Denis)(PIDF = Proportional-Integral-Derivative-Feedforward - a feedback system that makes your flywheel motors spin at exactly the target speed (55 RPS) with lightning-fast recovery.)
        //P=0.8: Aggressive speedup when slow
        //
        //I=0.0: No steady error (velocity PIDF rarely needs it)
        //
        //D=0.1: Light damping (prevents minor wobble)
        //
        //F=12.8: Overcomes flywheel inertia/friction instantly
        //Your shooter reaches 55 RPS in 0.6 seconds instead of 3+ seconds. Shooter is always ready when driver presses fire button. Perfect consistency across battery voltage changes.
    //time for testing i guess

        this.runtime = runtime;
        this.telemetry = telemetry;
    }

    private boolean revMode = false;
    private int prevPos = 0;
    private double prevTime = 0;
    private double targetRPS = 0;
    private static final double SERVO_SPEED = 0.02;

    public void setRevMode(boolean mode) { revMode = mode; }

    public void shoot() {
        targetRPS = 55.0;  // ~3300 RPM
    }

    public void shootslow() {
        targetRPS = 45.0;  // Slower shot
    }

    public void shootRev() {
        targetRPS = -12.0; // Index reverse
    }

    public void shootDistance(double distanceInches) {
        double rps;
        if (distanceInches < 20) rps = 45;
        else if (distanceInches < 40) rps = 55;
        else rps = 65;
        targetRPS = revMode ? -rps : rps;
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
        targetRPS = 0;
    }

    public void updateVelocity() {
        // Shooter velocity control
        shooter.setVelocity(targetRPS);
        shooter2.setVelocity(targetRPS);

        // Check conditions
        double sensorDistance = ballSensor.getDistance(DistanceUnit.INCH);
        ballInPosition = sensorDistance < Constants.BALL_PRESENT_DISTANCE;  // Ball detected
        rpsReady = Math.abs(shooter.getVelocity()) >= (targetRPS * 0.95);   // 95% of target RPS

        // AUTO SHOOT CYCLE: Both conditions met → index 1 ball
        if (ballInPosition && rpsReady && !indexing) {
            indexing = true;
            indexTimer.reset();
            roundUp();  // Feed 1 ball
        }

        // Stop indexing after short pulse
        if (indexing && indexTimer.time() > 0.25) {
            indexing = false;
            roundStop();  // Ready for next ball
        }

        // Telemetry
        telemetry.addData("Target RPS", "%.1f", targetRPS);
        telemetry.addData("Actual RPS", "%.1f", shooter.getVelocity());
        telemetry.addData("Ball Ready", ballInPosition);
        telemetry.addData("RPS Ready", rpsReady);
        telemetry.addData("Indexing", indexing);
    }





    // SAFE servo control - reads current position first
    public void aimUp() {
        double currentPos = aimLeft.getPosition();
        double newPos = Math.min(0.85, currentPos + SERVO_SPEED);
        aimLeft.setPosition(newPos);
        aimRight.setPosition(1.0 - newPos);
    }

    public void aimDown() {
        double currentPos = aimLeft.getPosition();
        double newPos = Math.max(0.15, currentPos - SERVO_SPEED);
        aimLeft.setPosition(newPos);
        aimRight.setPosition(1.0 - newPos);
    }
    public void aimStop() {
        aimLeft.setPosition(aimLeft.getPosition());  // Hold current position
        aimRight.setPosition(aimRight.getPosition()); // Hold current position (brake)
    }



    private void safeServoCheck() {
        double leftPos = aimLeft.getPosition();
        double rightPos = aimRight.getPosition();

        if (leftPos < 0.1 || leftPos > 0.9 || rightPos < 0.1 || rightPos > 0.9) {
            aimLeft.setPosition(0.5);
            aimRight.setPosition(0.5);
            telemetry.addData("SERVO", "EMERGENCY RESET!");
        }
    }

    private double speedFromTagDist(double ty) {
        double distFactor = Math.cos((ty+30)*Math.PI/180);
        return distFactor * 0.6 + 0.3;
    }
}
