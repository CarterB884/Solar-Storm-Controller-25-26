//package org.firstinspires.ftc.teamcode;
//
//import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//@TeleOp(name = "newStarterBotTeleop", group = "StarterBot")
//public class NewstarterBotTeleop.java extends OpMode {
//    final double FEED_TIME_SECONDS = 0.20;
//    final double STOP_SPEED = 0.0;
//    final double FULL_SPEED = 1.0;
//    final double LAUNCHER_TARGET_VELOCITY = 1125;
//    final double LAUNCHER_MIN_VELOCITY = 1075;
//
//    private DcMotor frontleftDrive = null;
//    private DcMotor frontrightDrive = null;
//    private DcMotor backrightDrive = null;
//    private DcMotor backleftDrive = null;
//    private DcMotorEx launcher = null;
//    private CRServo leftFeeder = null;
//    private CRServo rightFeeder = null;
//    ElapsedTime feederTimer = new ElapsedTime();
//
//    private enum LaunchState {
//        IDLE,
//        SPIN_UP,
//        LAUNCH,
//        LAUNCHING,
//    }
//
//    private LaunchState launchState;
//    double leftPower;
//    double rightPower;
//
//    @Override
//    public void init() {
//        launchState = LaunchState.IDLE;
//
//        frontleftDrive = hardwareMap.get(DcMotor.class, "leftdrive");
//        frontrightDrive = hardwareMap.get(DcMotor.class, "rightdrive");
//        backleftDrive = hardwareMap.get(DcMotor.class, "backleft");
//        backrightDrive = hardwareMap.get(DcMotor.class, "backright");
//        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
//        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
//        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
//
//        frontleftDrive.setDirection(DcMotor.Direction.REVERSE);
//        frontrightDrive.setDirection(DcMotor.Direction.FORWARD);
//        backleftDrive.setDirection(DcMotor.Direction.REVERSE);
//        backrightDrive.setDirection(DcMotor.Direction.FORWARD);
//
//        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        frontleftDrive.setZeroPowerBehavior(BRAKE);
//        frontrightDrive.setZeroPowerBehavior(BRAKE);
//        backleftDrive.setZeroPowerBehavior(BRAKE);
//        backrightDrive.setZeroPowerBehavior(BRAKE);
//        launcher.setZeroPowerBehavior(BRAKE);
//
//        leftFeeder.setPower(STOP_SPEED);
//        rightFeeder.setPower(STOP_SPEED);
//
//        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
//
//        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        telemetry.addData("Status", "Initialized");
//    }
//
//    @Override
//    public void init_loop() {}
//
//    @Override
//    public void start() {}
//
//    @Override
//    public void loop() {
//        // Driving logic
//        Mecandrive(gamepad1);
//
//        // Launcher controls
//        if (gamepad1.y) {
//            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
//        } else if (gamepad1.b) {
//            launcher.setVelocity(STOP_SPEED);
//        }
//
//        // Shot request - example, you may need to implement 'rightBumperWasPressed'
//        boolean shotRequested = gamepad1.right_bumper;
//
//        launch(shotRequested);
//
//        telemetry.addData("State", launchState);
//        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
//        telemetry.addData("motorSpeed", launcher.getVelocity());
//    }
//
//    private void launch(boolean shotRequested) {
//        switch (launchState) {
//            case IDLE:
//                if (shotRequested) {
//                    launchState = LaunchState.SPIN_UP;
//                }
//                break;
//            case SPIN_UP:
//                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
//                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
//                    launchState = LaunchState.LAUNCH;
//                }
//                break;
//            case LAUNCH:
//                leftFeeder.setPower(FULL_SPEED);
//                rightFeeder.setPower(FULL_SPEED);
//                feederTimer.reset();
//                launchState = LaunchState.LAUNCHING;
//                break;
//            case LAUNCHING:
//                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
//                    launchState = LaunchState.IDLE;
//                    leftFeeder.setPower(STOP_SPEED);
//                    rightFeeder.setPower(STOP_SPEED);
//                }
//                break;
//        }
//    }
//
//    // Correct drive method (no nested methods or loops)
//    private void Mecandrive(Gamepad gamepad1) {
//        double max;
//        double axial   = -gamepad1.left_stick_y;
//        double lateral =  gamepad1.left_stick_x;
//        double yaw     =  gamepad1.right_stick_x;
//
//        double frontLeftPower  = axial + lateral + yaw;
//        double frontRightPower = axial - lateral - yaw;
//        double backLeftPower   = axial - lateral + yaw;
//        double backRightPower  = axial + lateral - yaw;
//
//        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
//        max = Math.max(max, Math.abs(backLeftPower));
//        max = Math.max(max, Math.abs(backRightPower));
//
//        if (max > 1.0) {
//            frontLeftPower  /= max;
//            frontRightPower /= max;
//            backLeftPower   /= max;
//            backRightPower  /= max;
//        }
//
//        frontleftDrive.setPower(frontLeftPower);
//        frontrightDrive.setPower(frontRightPower);
//        backleftDrive.setPower(backLeftPower);
//        backrightDrive.setPower(backRightPower);
//
//        // Save for telemetry
//        leftPower = frontLeftPower;
//        rightPower = frontRightPower;
//    }
//
//    @Override
//    public void stop() {}
//}
