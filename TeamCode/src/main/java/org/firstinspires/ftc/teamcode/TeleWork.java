package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit.RADIANS;

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




@TeleOp
public class TeleWork extends OpMode {
    private DriveBase driveBase = null;
    private Shooter shooter = null;
    private Intake intake = null;
    public ElapsedTime runtime = new ElapsedTime();
    private GoBildaPinpointDriver goBildaPinpointDriver = null;
    private double targetHeading = 0.0;  // Goal angle in degrees
    private double ROTATION_KP = 0.015;  // Tune 0.01-0.02
    private boolean autoRotateActive = false;
    private boolean revOn = false;  // Reverse mode enabled by Y
    private Limelight3A limelight3A = null;
    boolean prevY = false;
    boolean prevB = false;




    @Override
    public void init() {
        goBildaPinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, Constants.ODOMETRY);


        // 2 args only (xOffset, yOffset in mm)
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
        driveBase.drive(gamepad1);
//        // Test heading for FOD
//        double headingDeg = goBildaPinpointDriver.getHeading(UnnormalizedAngleUnit.DEGREES);
//        telemetry.addData("Heading", "%.1f°", headingDeg);
//
//
        if (gamepad1.left_trigger > 0.5) {
            autoRotateActive = true;
//
            LLResult result = limelight3A.getLatestResult();
            if (result != null && result.isValid()) {
//                double tx = result.getTx();   // degrees: + means target to the right (usually)
                Pose3D botpose = result.getBotpose();
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("Botpose", botpose.toString());
            }
//                // Deadband so it doesn't jitter when nearly aligned
//                if (Math.abs(tx) < 1.0) tx = 0;
//
//                // P-control (tune kP)
//                double kP = 0.02; // start 0.015–0.03
//                double rotationPower = tx * kP;
//
//                // clamp
//                rotationPower = Math.max(-0.35, Math.min(0.35, rotationPower));
//
//                // If it turns the wrong way, flip the sign:
//                // rotationPower = -rotationPower;
//
//                driveBase.autoRotate(rotationPower);
//
//                telemetry.addData("LL tx", tx);
//                telemetry.addData("rotPwr", rotationPower);
//            }
//            else {
//                driveBase.autoRotate(0);
//                telemetry.addData("Limelight", "No target");
//            }
//
//        }
//        else {
//            autoRotateActive = false;
//
//            // ROBOT-CENTRIC drive (reliable baseline)
//            double axial = gamepad1.left_stick_y;
//            double lateral = gamepad1.left_stick_x;
//            double yaw = gamepad1.right_stick_x;
//
//            driveBase.drive(axial, lateral, yaw);
        }
//        // FOD
////        else {
////            autoRotateActive = false;
////            driveBase.fieldRelativeDrive(gamepad1);
////        }
//
//
//
//
////shooter-------------------------------------------------------------------------------------

//        boolean yPressed = gamepad2.yWasPressed();
//        if (yPressed) revOn = !revOn;




        if (gamepad2.right_bumper) {
            // Set direction
//            if (revOn) {
            shooter.shoot();

//            else {
//                shooter.shootRev();
//            }

        }
        else if (gamepad2.left_bumper) {
            shooter.shootslow();
        }
        else shooter.stop();
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
//
////old code------------------------------------------------

//        //old code in case new one doesn't work
//        // if (gamepad1.right_bumper){
//        //     shooter.shoot();
//        //        }
//        // else if (gamepad1.left_bumper){
//        //          shooter.shootRev();
//        // else {
//        //  shooter.stop();
////old code-------------------------------------------------
//// intake-------------------------------------------------------------------------------------------
        if (gamepad2.right_trigger > 0.5){
//            if (revOn) {
                intake.spinIn();
//            }
//            else {
//                intake.spinIn();

        }
        else {
            intake.spinStop();
        }
//// roundabout---------------------------------------------------------------------------------------
        if (gamepad2.dpad_up) {
                shooter.roundUp();
        }
        else if (gamepad2.dpad_down) {
            shooter.roundDown();
        }
        else shooter.roundStop();
//        //resets heading-----------------------------------------------------------------------
        if (gamepad1.a) {
            goBildaPinpointDriver.recalibrateIMU();
        }
//
//
//
//
//        telemetry.addData("Mode", autoRotateActive ? "AUTO-ROTATE" : "MANUAL");
//        telemetry.addData("Heading (deg)", headingDeg);
//        telemetry.addData("X", "%.1f\"", goBildaPinpointDriver.getPosX(DistanceUnit.INCH));
//        telemetry.addData("Y", "%.1f\"", goBildaPinpointDriver.getPosY(DistanceUnit.INCH));
//        telemetry.addData("LT", gamepad1.left_trigger);
//

        telemetry.update();

//        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }


}

