package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Constants {
    public static final String SHOOT = "shooter";
    public static final String FRONT_LEFT = "fl";
    public static final String FRONT_RIGHT = "fr";
    public static final String BACK_LEFT = "bl";
    public static final String BACK_RIGHT = "br";
    public static final String INTAKE = "in";
    public static final String ROUNDABOUT = "round";
    public static final String ODOMETRY = "odey";
    public static final String SHOOTER2 = "shooter2";  // Second flywheel motor
    public static final String AIM_LEFT = "aimLeft";   // Left torque servo
    public static final String AIM_RIGHT = "aimRight"; // Right torque servo
    public static final String BALL_SENSOR = "ballSensor";  // Distance sensor name
    public static final double BALL_PRESENT_DISTANCE = 4;  // Inches = ball detected
    private boolean shootCommanded = false;    // Driver is holding shoot
    private boolean ballWasSeen = false;       // For intake packing timing
    private ElapsedTime packTimer = new ElapsedTime();


}
