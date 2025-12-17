package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.ArrayList;
import java.util.List;

/**
 * Utility to draw robot pose on FTC Dashboard field overlay.
 */
public final class display {

    private static final double ROBOT_RADIUS_IN = 6.0; // half-length for the arrow
    private static final int TRAIL_LENGTH = 100;

    private static final List<Pose2d> trail = new ArrayList<>();

    private display() {}

    public static void sendPose(final Pose2d pose) {
        sendPoseWithGhost(pose, null);
    }

    public static void sendPoseWithGhost(final Pose2d pose, final Pose2d ghost) {
        if (pose == null) {
            return;
        }
        trail.add(pose);
        if (trail.size() > TRAIL_LENGTH) {
            trail.remove(0);
        }

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("poseX_in", pose.getX());
        packet.put("poseY_in", pose.getY());
        packet.put("poseHeading_deg", Math.toDegrees(pose.getHeading()));

        drawArrow(packet, pose, "#00FFFF");
        if (ghost != null) {
            drawArrow(packet, ghost, "#FF66FF");
            packet.put("LL_poseX_in", ghost.getX());
            packet.put("LL_poseY_in", ghost.getY());
            packet.put("LL_poseHeading_deg", Math.toDegrees(ghost.getHeading()));
        } else {
            packet.put("LL_poseX_in", 0);
            packet.put("LL_poseY_in", 0);
            packet.put("LL_poseHeading_deg", 0);
        }

        if (trail.size() > 1) {
            double[] xs = new double[trail.size()];
            double[] ys = new double[trail.size()];
            for (int i = 0; i < trail.size(); i++) {
                xs[i] = trail.get(i).getX();
                ys[i] = trail.get(i).getY();
            }
            packet.fieldOverlay()
                    .setStroke("#FF9900")
                    .strokePolyline(xs, ys);
        }

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    private static void drawArrow(final TelemetryPacket packet, final Pose2d pose, final String colorHex) {
        final double h = pose.getHeading();
        final double x = pose.getX();
        final double y = pose.getY();
        final double r = ROBOT_RADIUS_IN;

        final double hx = x + r * Math.cos(h);
        final double hy = y + r * Math.sin(h);
        final double lx = x - r * Math.cos(h) - r * Math.sin(h);
        final double ly = y - r * Math.sin(h) + r * Math.cos(h);
        final double rx = x - r * Math.cos(h) + r * Math.sin(h);
        final double ry = y - r * Math.sin(h) - r * Math.cos(h);

        packet.fieldOverlay()
                .setStroke(colorHex)
                .strokePolyline(
                        new double[]{hx, lx, rx, hx},
                        new double[]{hy, ly, ry, hy}
                );
    }
}


