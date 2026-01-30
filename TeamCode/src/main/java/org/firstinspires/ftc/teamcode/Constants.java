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
    public static final String ODOMETRY = "odey";  // Your GoBilda
    public static final String SHOOTER2 = "shooter2";
    public static final String AIM_LEFT = "aimLeft";
    public static final String AIM_RIGHT = "aimRight";
    public static final String BALL_SENSOR = "ballSensor";
    public static final double BALL_PRESENT_DISTANCE = 3;
    public static final double targetRPS = 0;

    private boolean shootCommanded = false;
    private boolean ballWasSeen = false;
    private ElapsedTime packTimer = new ElapsedTime();
}
