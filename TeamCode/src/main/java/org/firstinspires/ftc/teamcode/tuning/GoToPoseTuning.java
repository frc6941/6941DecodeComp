package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.Constants;

@Config
public final class GoToPoseTuning {

    public static double TRANSLATION_kP = Constants.Drive.GOTO_POSE_X_kP;
    public static double TRANSLATION_kI = Constants.Drive.GOTO_POSE_X_kI;
    public static double TRANSLATION_kD = Constants.Drive.GOTO_POSE_X_kD;

    public static double TURN_kP = Constants.Drive.GOTO_POSE_TURN_kP;
    public static double TURN_kI = Constants.Drive.GOTO_POSE_TURN_kI;
    public static double TURN_kD = Constants.Drive.GOTO_POSE_TURN_kD;

    public static double HEADING_TOLERANCE_DEG = Constants.Drive.GOTO_POSE_HEADING_TOLERANCE_DEG;
    public static double MAX_TRANSLATION_OUTPUT = Constants.Drive.GOTO_POSE_MAX_TRANSLATION_OUTPUT;
    public static double MAX_TURN_OUTPUT = Constants.Drive.GOTO_POSE_MAX_TURN_OUTPUT;

    public static double ROTATION_ADJUSTMENT_MAX_DEG = 30.0;

    private GoToPoseTuning() {
    }
}
