package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit.RADIANS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Constants;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Mechanism.Intake;
import org.firstinspires.ftc.teamcode.Mechanism.Shooter;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Constants;
import com.qualcomm.hardware.limelightvision.LLResult;




@TeleOp(name="TeleWork", group="Robot")
public class TeleWork extends OpMode {
    private DriveBase driveBase = null;
    private Shooter shooter = null;
    private Intake intake = null;
    public ElapsedTime runtime = new ElapsedTime();
    private GoBildaPinpointDriver goBildaPinpointDriver = null;
    private double targetHeading = 0.0;
    private double ROTATION_KP = 0.015;
    private boolean autoRotateActive = false;
    private Limelight3A limelight3A = null;
    public static double limelightYawAssist = 0;  // Stored PID yaw

    private ElapsedTime pidTimer = new ElapsedTime();
    private double kp = 0.035;
    private double ki = 0.008;
    private double kd = 0.08;



//    boolean prevY = false;
//    boolean prevB = false;
//    private boolean revOn = false;





    @Override
    public void init() {
        goBildaPinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, Constants.ODOMETRY);



        goBildaPinpointDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        goBildaPinpointDriver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        goBildaPinpointDriver.setOffsets(-7.48, -8.46, DistanceUnit.INCH);  // Your pod measurements

        shooter = new Shooter(hardwareMap, runtime, telemetry);
        driveBase = new DriveBase(hardwareMap, goBildaPinpointDriver);
        intake = new Intake(hardwareMap);
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);

    }


    public void start() {
        runtime.reset();
        limelight3A.start();
        goBildaPinpointDriver.recalibrateIMU();
    }

    @Override
    public void loop() {
        goBildaPinpointDriver.update();

        if (gamepad1.left_trigger > 0.5) {
            autoRotateActive = true;
            LLResult result = limelight3A.getLatestResult();

            if (result != null && result.isValid()) {
                double tx = result.getTx();
                tx = tx - 4;

                if (Math.abs(tx) < 1.0) {
                    driveBase.fieldRelativeDrive(gamepad1);
                } else {
                    double kP = 0.025;
                    double rotationPower = tx * kP;
                    rotationPower = Math.max(-0.35, Math.min(0.35, rotationPower));

                    // **COMBINES stick driving + auto rotation**
                    driveBase.fieldRelativeDriveWithYaw(gamepad1, rotationPower);
                    telemetry.addData("tx°", "%.1f", tx);
                    telemetry.addData("rotPwr", "%.2f", rotationPower);
                }
            } else {
                driveBase.fieldRelativeDrive(gamepad1);
                telemetry.addData("Limelight", "No tag");
            }
        } else {
            autoRotateActive = false;
            driveBase.fieldRelativeDrive(gamepad1);
        }










        //        //auto shoot-------------------------------------------------------------------------
//        else if (gamepad1.dpad_left) {
//            shooter.setRevMode(revOn);  // Set direction
//
//            double distanceInches = 40.0;  // Default safe distance
//            try {
//                double robotX = goBildaPinpointDriver.getPosX(DistanceUnit.INCH);
//                double robotY = goBildaPinpointDriver.getPosY(DistanceUnit.INCH);
//                distanceInches = Math.min(Math.sqrt(robotX * robotX + robotY * robotY), 75.0);
//            } catch (Exception e) {
//                distanceInches = 40.0;  // Fallback
//            }
//            shooter.shootDistance(distanceInches);
//        }
//
//        else {
//            shooter.stop();
//        }
//
////old code------------------------------------------------
//
//        //old code in case new one doesn't work
//        // if (gamepad1.right_bumper){
//        //     shooter.shoot();
//        //        }
//       // else if (gamepad1.left_bumper){
//       //          shooter.shootRev();
//        // else {
//       //  shooter.stop();
////old code-------------------------------------------------
//shooter-------------------------------------------------------------------------------------
        boolean shootHeld = gamepad1.right_bumper;
        if (shootHeld) {
            shooter.shoot();
            shooter.setShootCommanded(true);
        } else if (gamepad1.left_bumper) {
            shooter.shootslow(shooter.targetRPS);
            shooter.setShootCommanded(true);
        } else {
            shooter.stop();
            shooter.setShootCommanded(false);
        }
//limelight testing---------------------------------------------------------------------------------
// AUTO SHOOTER AIM - Gamepad2 X (uses ty for distance)
        if (gamepad1.x) {
            LLResult result = limelight3A.getLatestResult();
            if (result != null && result.isValid()) {
                double ty = result.getTy();

                // Distance estimate from ty (higher ty = closer)
                double distanceEstimate = 120 - (ty * 8);  // Tune these numbers!

                shooter.updateAimFromVision(ty);
                shooter.shootslow(distanceEstimate);  // Uses distanceEstimate

                telemetry.addData("Auto Aim", "ON");
                telemetry.addData("ty°", "%.1f", ty);
                telemetry.addData("Est Dist", "%.1f in", distanceEstimate);
                telemetry.addData("AimPos", "%.3f", shooter.getTargetAimPos());
            }
        }

//--------------------------------------------------------------------------------------------------
//// intake-----------------------------------------------------------------------------------------
        if (gamepad1.right_trigger > 0.5){
//            if (revOn) {
                intake.spinIn();
//            }
//            else {
//                intake.spinIn();

        }
        else {
            intake.spinStop();
        }
// roundabout---------------------------------------------------------------------------------------
        if (gamepad1.dpad_up) {
                shooter.roundUp();
        }
        else if (gamepad1.dpad_down) {
            shooter.roundDown();
        }
        else shooter.roundStop();
//      resets heading--------------------------------------------------------------------------------------
        if (gamepad1.a) {
            goBildaPinpointDriver.setHeading(0, AngleUnit.DEGREES);
        }
        //aiming------------------------------------------------------------------------------------
        if (gamepad1.dpadRightWasPressed()) {
            shooter.aimDown();
        }
        else if (gamepad1.dpadLeftWasPressed()) {
            shooter.aimUp();
        }

//--------------------------------------------------------------------------------------------------
        if (gamepad1.b) {
            shooter.aimRight.setPosition(0.5);
        }
//
//        //resets heading-----------------------------------------------------------------------
//        telemetry.addData("Mode", autoRotateActive ? "AUTO-ROTATE" : "MANUAL");
//        telemetry.addData("Heading (deg)", headingDeg);
//        telemetry.addData("X", "%.1f\"", goBildaPinpointDriver.getPosX(DistanceUnit.INCH));
//        telemetry.addData("Y", "%.1f\"", goBildaPinpointDriver.getPosY(DistanceUnit.INCH));
//        telemetry.addData("LT", gamepad1.left_trigger);
//
        shooter.updateVelocity();
        telemetry.update();

//        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }


}

