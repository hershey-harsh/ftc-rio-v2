package org.firstinspires.ftc.teamcode;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.FusionLocalizer;

public class Configuration {
    public static final String RIGHT_FRONT_MOTOR = "motor1";
    public static final String LEFT_FRONT_MOTOR = "motor0";
    public static final String RIGHT_REAR_MOTOR = "motor3";
    public static final String LEFT_REAR_MOTOR = "motor2";
    public static final String ODOMETRY_POD = "odo";
    public static final String LIMELIGHT = "limelight";
    public static final String TURRET_SERVO_LEFT = "servo0B"; // servo2B
    public static final String TURRET_SERVO_RIGHT = "servo5B"; // servo3B
    public static final String TRANSFER_MOTOR_ONE = "motor3B";
    public static final String TRANSFER_MOTOR_TWO = "motor2B";
    public static final String GATE_3_SENSOR = "color";
    public static final String SERVO_GATE_LEFT = "servo2B"; // servo0B
    public static final String SERVER_GATE_RIGHT = "servo1B";
    public static final String HOOD_SERVO_RIGHT = "servo4B";
    public static final String HOOD_SERVO_LEFT = "servo3B"; //servo%B
    public static final String ROBOT_LIGHT = "servo5";
    public static final String TURRET_LIGHT = "servo4";
    public static final String RIGHT_TURRET_MOTOR = "motor0B";
    public static final String LEFT_TURRET_MOTOR = "motor1B";

    public static FusionLocalizer fusionLocalizer = null;
    public static Pose CURRENT_POSE = new Pose();

    public static final Pose RED_GOAL_POSE = new Pose(144, 144);
    public static final Pose BLUE_GOAL_POSE = new Pose(0, 144);
    public static final Pose MANUAL_LOCALIZATION_POSE = new Pose(72, 72, Math.toRadians(270));
    public static double X_GOAL_OFFSET = 0, Y_GOAL_OFFSET = 0, TURRET_OFFSET = 0;

    public static void setAimPointOffset(double xOffset, double yOffset) {
        X_GOAL_OFFSET = xOffset;
        Y_GOAL_OFFSET = yOffset;
    }

    public static double ARTIFACT_TRANSFER_TIME = 0.7;
    public static double SHOOTER_HEIGHT_TO_GOAL = 1.1;
    public static double VELOCITY_COMPENSATION_WEIGHT = 0.3; // how much to adjust hood angle based on shooter wheel velocity
    public static double RPM_MULTIPLER = 2; // adjust this to fine-tune shooter wheel RPM without changing the LUT
    public static double SHOOTER_TIME = 0.6;


    public static int RED_LIMELIGHT_PIPELINE = 0;
    public static int BLUE_LIMELIGHT_PIPELINE = 1;

    // Configure Alliance and Control Scheme between and prior to matches.

    public enum Alliance {RED, BLUE}
    public static Alliance ALLIANCE = Alliance.RED;
    public static boolean FIELD_CENTRIC = false;
    public static double CONTROL_SCALE = 1.0;
}