package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.DriveBase;
import org.firstinspires.ftc.teamcode.Mechanism.Intake;
import org.firstinspires.ftc.teamcode.Mechanism.Shooter;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Robot: Auto blue front", group="Robot")
public class Autobluefront extends OpMode {

    private DriveBase driveBase = null;
    private Shooter shooter = null;
    private Intake intake = null;
    private GoBildaPinpointDriver goBildaPinpointDriver = null;
    private Limelight3A limelight3A = null;
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor frontLeftDrive = null;
    public DcMotor backLeftDrive = null;
    public DcMotor frontRightDrive = null;
    public DcMotor backRightDrive = null;

    private int autoStep = 0;

    @Override
    public void init() {
        // ALL YOUR ORIGINAL INIT CODE - UNCHANGED
        goBildaPinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, Constants.ODOMETRY);
        goBildaPinpointDriver.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        goBildaPinpointDriver.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        goBildaPinpointDriver.setOffsets(-7.48, -8.46, DistanceUnit.INCH);

        shooter = new Shooter(hardwareMap, runtime, telemetry);
        driveBase = new DriveBase(hardwareMap, goBildaPinpointDriver);
        intake = new Intake(hardwareMap);

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);

        frontLeftDrive = hardwareMap.get(DcMotor.class, Constants.FRONT_LEFT);
        frontRightDrive = hardwareMap.get(DcMotor.class, Constants.FRONT_RIGHT);
        backLeftDrive = hardwareMap.get(DcMotor.class, Constants.BACK_LEFT);
        backRightDrive = hardwareMap.get(DcMotor.class, Constants.BACK_RIGHT);
    }
    // ADD THESE FUNCTIONS TO YOUR Autobluefront CLASS (keep everything else exactly the same)

    private void shootslow(double distanceInches) {
        shooter.shootslow(distanceInches);
        shooter.setShootCommanded(true);
    }

    private void forwardandintake(double seconds) {
        intake.spinIn();
        setDrivePower(0.5, 0.5, 0.5, 0.5);
        // You handle timing in autoStep
    }

    private void goforward(double seconds) {
        setDrivePower(0.5, 0.5, 0.5, 0.5);
    }

    private void gobackward(double seconds) {
        setDrivePower(-0.5, -0.5, -0.5, -0.5);
    }

    private void turnL(double seconds) {
        setDrivePower(-0.3, -0.3, 0.3, 0.3);
    }

    private void turnR(double seconds) {
        setDrivePower(0.3, 0.3, -0.3, -0.3);
    }

    private void stopDrive() {
        setDrivePower(0, 0, 0, 0);
    }


    @Override
    public void start() {
        runtime.reset();
        autoStep = 0;
        if (limelight3A != null) limelight3A.start();
        if (goBildaPinpointDriver != null) goBildaPinpointDriver.recalibrateIMU();
    }

    @Override
    public void loop() {
        if (goBildaPinpointDriver != null) {
            goBildaPinpointDriver.update();
        }
        shooter.updateVelocity();

        switch (autoStep) {
            case 0:
                if (runtime.seconds() < 1.3) {
                    gobackward(1.3);
                } else {
                    stopDrive();
                    runtime.reset();
                    autoStep++;
                }
                break;

            case 1:
                if (runtime.seconds() < 7.0) {
                    shootslow(60);
                    shooter.setShootCommanded(true);
                } else {
                    shooter.stop();
                    shooter.setShootCommanded(false);
                    runtime.reset();
                    autoStep++;
                }
                break;

            case 2:
                if (runtime.seconds() < 0.495) {
                    turnL(0.495);
                } else {
                    stopDrive();
                    runtime.reset();
                    autoStep++;
                }
                break;

            case 3:
                if (runtime.seconds() < 1.5) {
                    forwardandintake(1.5);
                } else {
                    intake.spinStop();
                    stopDrive();
                    runtime.reset();
                    autoStep++;
                }
                break;
            case 4:
                if (runtime.seconds() < 1.5) {
                    gobackward(1.5);
                } else {
                    stopDrive();
                    runtime.reset();
                    autoStep++;
                }
                break;

            case 5:
                if (runtime.seconds() < 1.5) {
                    shootslow(60);
                    shooter.setShootCommanded(true);
                } else {
                    shooter.stop();
                    shooter.setShootCommanded(false);
                    runtime.reset();
                    autoStep++;
                }
                break;
        }
    }

        // YOUR ORIGINAL FUNCTIONS - BULLETPROOF VERSION
    private void setDrivePower(double fl, double bl, double fr, double br) {
        // DIRECT MOTOR CONTROL - BYPASSES ALL DriveBase PROBLEMS
        frontLeftDrive.setPower(fl);
        backLeftDrive.setPower(bl);
        frontRightDrive.setPower(fr);
        backRightDrive.setPower(br);
    }

}
