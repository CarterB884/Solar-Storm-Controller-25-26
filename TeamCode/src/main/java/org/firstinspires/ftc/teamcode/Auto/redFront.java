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

@Autonomous(name="Robot: Auto red front", group="Robot")
public class redFront extends OpMode {

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
    private double currentServoPos = 0.5;
    //private boolean rpsReady = false;

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


    /*private void autoRotateToTag() {
        if (limelight3A == null) {
            stopDrive();
            return;
        }

        LLResult result = limelight3A.getLatestResult();


        if (result != null && result.isValid()) {
            double tx = result.getTx();
            tx = tx - 7;

            if (Math.abs(tx) < 1.0) {
                stopDrive();
                telemetry.addData("Tag", "Centered!");
            } else {
                double kP = 0.025;
                double rotationPower = tx * kP;
                rotationPower = Math.max(-0.35, Math.min(0.35, rotationPower));

                driveBase.autoRotate(rotationPower);
                telemetry.addData("tx°", "%.1f", tx);
                telemetry.addData("rotPwr", "%.2f", rotationPower);
            }
        } else {
            telemetry.addData("Limelight", "No valid tag");
        }
    }*/





    private void forwardandintake(double seconds) {
        intake.spinIn();
        setDrivePower(0.5, 0.5, 0.5, 0.5);
        // You handle timing in autoStep
    }
    private void intakeAuto(double seconds){
        intake.spinIn();
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
    private void moveL(double seconds){
        setDrivePower(-0.6, 0.6, 0.6, -0.6);
    }
    private void moveR(double seconds){
        setDrivePower(0.6, -0.6, -0.6, 0.6);
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


        switch (autoStep) {
            case 0:
                shooter.aimRight.setPosition(0.52);
                if (runtime.seconds() < 1.55) {
                    gobackward(1.55);
                } else {
                    stopDrive();
                    runtime.reset();
                    autoStep++;
                }
                break;
            case 1:
                if (runtime.seconds() < 0.25){
                    moveR(0.25);
                } else {
                    stopDrive();
                    runtime.reset();
                    autoStep++;
                }
                break;
            case 2:
                if (runtime.seconds() < 0.05) {
                    turnL(0.05);
                } else {
                    stopDrive();
                    runtime.reset();
                    autoStep++;
                }
                break;


            case 3:  // 3 BALLS - WAIT FOR SPEED - Commented out for now so I don't have to worry about shooting
                if (runtime.seconds() < 5) {
                    shooter.prepareVerySlowShot();

                    // 🔥 ONLY SHOOT WHEN READY
//                    if (!shooter.rpsReady) {
//                        telemetry.addData("Waiting", "Flywheel speed...");
//                    } else {
//                        telemetry.addData("RPS Ready", "SHOOTING!");
//                    }
                } else {
                    shooter.setFlywheelSpeed0();
                    shooter.setShootCommanded(false);
                    runtime.reset();
                    autoStep++;
                }
                break;

            case 4:
                if (runtime.seconds() < 0.6) {
                    turnR(0.6);
                } else {
                    stopDrive();
                    runtime.reset();
                    autoStep++;
                }
                break;

            case 5:
                if (runtime.seconds() < 0.2){
                    moveR(0.2);
                } else {
                    stopDrive();
                    runtime.reset();
                    autoStep++;
                }
                break;

            case 6:
                if (runtime.seconds() < 1.5) {
                    forwardandintake(1.5);
                } else {
                    stopDrive();
                    runtime.reset();
                    autoStep++;
                }
                break;

            case 7:
                if (runtime.seconds() < .5) {
                    intakeAuto(.5);
                } else {
                    stopDrive();
                    runtime.reset();
                    autoStep++;
                }
                break;

            case 8:
                if (runtime.seconds() < 1.45) {
                    gobackward(1.45);
                } else {
                    stopDrive();
                    runtime.reset();
                    autoStep++;
                }
                break;

            case 9:
                shooter.aimRight.setPosition(0.52);
                if (runtime.seconds() < 0.49) {
                    turnL(0.49);
                } else {
                    stopDrive();
                    runtime.reset();
                    autoStep++;
                }

            case 10:

                if (runtime.seconds() < 5) {


                    shooter.prepareVerySlowShot();

                    // 🔥 ONLY SHOOT WHEN READY
//                    if (!shooter.rpsReady) {
//                        telemetry.addData("Waiting", "Flywheel speed...");
//                    } else {
//                        telemetry.addData("RPS Ready", "SHOOTING!");
//                    }
                } else {
                    shooter.setFlywheelSpeed0();
                    shooter.setShootCommanded(false);
                    runtime.reset();
                    autoStep++;
                }
                break;
            case 11:
                if (runtime.seconds() < 0.59) {
                    moveR(0.5);
                } else {
                    stopDrive();
                    runtime.reset();
                    autoStep++;
                }
        }
        shooter.update();
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
