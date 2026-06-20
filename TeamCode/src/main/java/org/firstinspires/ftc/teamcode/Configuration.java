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
    public static final String GATE_1_AND_2_SENSOR = "digital2B";

    public static FusionLocalizer fusionLocalizer = null;
    public static Pose CURRENT_POSE = new Pose();

    public static final Pose RED_GOAL_POSE = new Pose(144, 144);
    public static final Pose BLUE_GOAL_POSE = new Pose(0, 144);
    public static final Pose MANUAL_LOCALIZATION_POSE = new Pose(72, 72, Math.toRadians(270));
    public static final Pose RED_LOCALIZATION_POSE = new Pose(8.045, 7.844, Math.toRadians(180));
    public static final Pose BLUE_LOCALIZATION_POSE = new Pose(133.954, 8.130272727272725, Math.toRadians(0));
    public static final Pose GATE_OPEN_RED = new Pose(131.752, 62.388, Math.toRadians(23.19));
    public static final Pose GATE_OPEN_RED_2 = new Pose(111.45428282828283, 53.34450505050505);
    public static final Pose GATE_OPEN_BLUE = new Pose(10.5, 58.598, Math.toRadians(151.84));
    public static final Pose GATE_OPEN_BLUE_2 = new Pose(35.716, 55.717); //TODO: Fine tune.
    public static final Pose RED_PARK_BR = new Pose(38.565656565656575, 32.29090909090909, Math.toRadians(90));
    public static final Pose BLUE_PARK_BR = new Pose(39.70909090909092, 32.29090909090909, Math.toRadians(90)).mirror();
    public static final Pose RED_LAUNCH = new Pose(39.70909090909092, 32.29090909090909, Math.toRadians(90));
    public static double X_GOAL_OFFSET = 0, Y_GOAL_OFFSET = 0, TURRET_OFFSET = 0;
    // Predictive shoot-on-the-move turret lead (deg), set by AimController. Composes with
    // (does not replace) the mechanical TURRET_OFFSET. Zero when not actively leading.
    public static double TURRET_PREDICTIVE_OFFSET = 0;
    // Zone-based turret offset (deg), set by AimController when inside OFFSET_ZONE. Composes
    // with the other offsets. -3 for RED, +3 for BLUE; 0 elsewhere.
    public static double TURRET_ZONE_OFFSET = 0;

    // ---- SOTM zones (RED field coordinates; BLUE is the pose.mirror() of each) -------------
    // "Offset Zone": while the robot is inside it, add a -3deg (RED) / +3deg (BLUE)
    // turret offset (the geometry/decision is applied in AimController).
    public static final Pose[] OFFSET_ZONE_RED = {
            new Pose(70.5530462, 117.6701680),
            new Pose(70.7715336, 140.559873),
            new Pose(1.101365546, 140.1838235),
            new Pose(0.0588235, 117.6286764)
    };
    // "Far Shoot Zone" (the tiny triangle): while inside it, aim at FAR_SHOOT_GOAL with NO
    // lead/offsets — a fixed long shot.
    public static final Pose[] FAR_SHOOT_ZONE_RED = {
            new Pose(24.1790966, 0.1003315126),
            new Pose(117.888655, 0.2489495),
            new Pose(70.959558, 41.0446428)
    };
    public static final Pose FAR_SHOOT_GOAL_RED = new Pose(135.2221638655462, 140.6386554621849);
    // "Close Shoot Zone" (RED; BLUE = mirror). The robot shoots from inside this triangle; the
    // competition teleop & gate-run auto-open the transfer gate on ENTRY into it. The apex Y was
    // lowered from the field editor's 65.945 to 50.0 so the shoot pose (~81.5, 76.1) and the
    // drive back from the gate fall well inside (the original apex sat just above the shoot pose,
    // so the robot skimmed the edge and never crossed in). Gate (y~58) stays outside, so the
    // exit->re-entry edge still fires.
    public static final Pose[] CLOSE_SHOOT_ZONE_RED = {
            new Pose(70.7016806, 50.0),
            new Pose(141.8188025, 135.952205),
            new Pose(0.2095588, 136.1706932)
    };
    // Far-shoot-zone turret offset for RED (negative = RIGHT/CW per the convention: +angle =
    // CCW/left, confirmed by the rotation-lead sign). BLUE uses the NEGATIVE of this, so RED
    // aims right and BLUE aims left. Flip a sign if a side goes the wrong way.
    public static double FAR_SHOOT_TURRET_OFFSET = -2.0;
    // Offset-zone turret offset magnitude (positive = LEFT/CCW). RED applies the NEGATIVE
    // (right) and BLUE the positive (left) — flipped because RED was over-aiming left.
    public static double OFFSET_ZONE_TURRET_OFFSET = 3.0;
    // Extra RPM multiplier added on top of RPM_MULTIPLER while in the far-shoot zone (hotter
    // wheel for the long shot). e.g. RPM_MULTIPLER=2 -> 2.3 in the far-shoot zone.
    public static double FAR_SHOOT_RPM_BONUS = 0.15;

    // BLUE = mirror of RED (pose.mirror() flips to the other alliance side).
    public static final Pose[] OFFSET_ZONE_BLUE = mirrorAll(OFFSET_ZONE_RED);
    public static final Pose[] FAR_SHOOT_ZONE_BLUE = mirrorAll(FAR_SHOOT_ZONE_RED);
    public static final Pose FAR_SHOOT_GOAL_BLUE = FAR_SHOOT_GOAL_RED.mirror();
    public static final Pose[] CLOSE_SHOOT_ZONE_BLUE = mirrorAll(CLOSE_SHOOT_ZONE_RED);

    private static Pose[] mirrorAll(Pose[] src) {
        Pose[] out = new Pose[src.length];
        for (int i = 0; i < src.length; i++) out[i] = src[i].mirror();
        return out;
    }

    /** True if (x,y) is inside the current alliance's 2-degree offset zone. */
    public static boolean inOffsetZone(double x, double y) {
        return pointInPolygon(ALLIANCE == Alliance.RED ? OFFSET_ZONE_RED : OFFSET_ZONE_BLUE, x, y);
    }

    /** True if (x,y) is inside the current alliance's far-shoot (tiny triangle) zone. */
    public static boolean inFarShootZone(double x, double y) {
        return pointInPolygon(ALLIANCE == Alliance.RED ? FAR_SHOOT_ZONE_RED : FAR_SHOOT_ZONE_BLUE, x, y);
    }

    /** True if (x,y) is inside the current alliance's close-shoot zone (gate auto-open trigger). */
    public static boolean inCloseShootZone(double x, double y) {
        return pointInPolygon(ALLIANCE == Alliance.RED ? CLOSE_SHOOT_ZONE_RED : CLOSE_SHOOT_ZONE_BLUE, x, y);
    }

    /** Current alliance's far-shoot aim point. */
    public static Pose farShootGoal() {
        return ALLIANCE == Alliance.RED ? FAR_SHOOT_GOAL_RED : FAR_SHOOT_GOAL_BLUE;
    }

    /** Current alliance's normal goal. */
    public static Pose allianceGoal() {
        return ALLIANCE == Alliance.RED ? RED_GOAL_POSE : BLUE_GOAL_POSE;
    }

    /** Standard ray-casting point-in-polygon test (field coordinates, inches). */
    public static boolean pointInPolygon(Pose[] poly, double x, double y) {
        boolean inside = false;
        for (int i = 0, j = poly.length - 1; i < poly.length; j = i++) {
            double xi = poly[i].getX(), yi = poly[i].getY();
            double xj = poly[j].getX(), yj = poly[j].getY();
            boolean intersect = ((yi > y) != (yj > y))
                    && (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
            if (intersect) inside = !inside;
        }
        return inside;
    }

    public static void setAimPointOffset(double xOffset, double yOffset) {
        X_GOAL_OFFSET = xOffset;
        Y_GOAL_OFFSET = yOffset;
    }

    public static double ARTIFACT_TRANSFER_TIME = 0.7;
    public static double SHOOTER_HEIGHT_TO_GOAL = 0.95;
    public static double VELOCITY_COMPENSATION_WEIGHT = 0.3; // how much to adjust hood angle based on shooter wheel velocity
    public static double RPM_MULTIPLER = 2; // adjust this to fine-tune shooter wheel RPM without changing the LUT
    public static double SHOOTER_TIME = 0.5;


    public static int RED_LIMELIGHT_PIPELINE = 2;
    public static int BLUE_LIMELIGHT_PIPELINE = 3;
    public static int MOTIF_LIMELIGHT_PIPELINE = 4;

    // Configure Alliance and Control Scheme between and prior to matches.

    public enum Alliance {RED, BLUE}
    public static Alliance ALLIANCE = Alliance.RED;
    public static boolean FIELD_CENTRIC = false;
    public static double CONTROL_SCALE = 1.0;
}