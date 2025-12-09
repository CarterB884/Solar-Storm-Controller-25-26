package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit.RADIANS;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Mechanism.Intake;
import org.firstinspires.ftc.teamcode.Mechanism.Shooter;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class TeleWork extends OpMode {
    private DriveBase driveBase = null;
    private Shooter shooter = null;
    private Intake intake = null;
    public ElapsedTime runtime = new ElapsedTime();
    private GoBildaPinpointDriver goBildaPinpointDriver = null;


    @Override
    public void init() {
        goBildaPinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, Constants.ODOMETRY);

        // Configure Pinpoint odometry - CORRECTED API
        goBildaPinpointDriver.setOffsets(0.0, 0.0, DistanceUnit.MM);

        // 2 args only (xOffset, yOffset in mm)
        goBildaPinpointDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        goBildaPinpointDriver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        shooter = new Shooter(hardwareMap, runtime, telemetry);
        driveBase = new DriveBase(hardwareMap, goBildaPinpointDriver);
        intake = new Intake(hardwareMap);
    }


    public void start() {
        runtime.reset();
        goBildaPinpointDriver.recalibrateIMU();
    }

    @Override
    public void loop() {
        goBildaPinpointDriver.update();

        driveBase.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        //shooter-----------------------------------------------------------------------------
        if (gamepad1.right_bumper){
            shooter.shoot();
        }
        else if (gamepad1.y) {
            if (gamepad1.right_bumper){
                shooter.shootRev();
            }
        }
        else {
            shooter.stop();
        }

        //old code in case new one doesn't work
        // if (gamepad1.right_bumper){
        //     shooter.shoot();
        //        }
        // else if (gamepad1.left_bumper){
        //          shooter.shootRev();
        // else {
        //  shooter.stop();

        //intake-------------------------------------------------------------------------------
        if (gamepad1.right_trigger > 0.5){
            intake.spinIn();
        }
        else {
            intake.spinStop();
        }
        // roundabout--------------------------------------------------------------------------
        if (gamepad1.dpad_up){
            shooter.roundUp();
        }
        else {
            shooter.roundStop();
        }
        //resets heading-----------------------------------------------------------------------
        if (gamepad1.a) {
            goBildaPinpointDriver.recalibrateIMU();
        }

        telemetry.addData("Heading (deg)",
                goBildaPinpointDriver.getHeading(UnnormalizedAngleUnit.DEGREES));
        telemetry.update();

//        telemetry.addData("Status", "Run Time: " + runtime.toString());
//
        telemetry.update();
    }


}

