package frc.robot.util;

import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;

public final class PolygonUtil {
    private PolygonUtil() {}

    public static boolean contains(List<Translation2d> polygon, Translation2d point) {
        boolean inside = false;

        for (int i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
            double xi = polygon.get(i).getX();
            double yi = polygon.get(i).getY();
            double xj = polygon.get(j).getX();
            double yj = polygon.get(j).getY();

            boolean intersects = ((yi > point.getY()) != (yj > point.getY()))
                && (point.getX() < (xj - xi) * (point.getY() - yi) / ((yj - yi) + 1e-9) + xi);

            if (intersects) {
                inside = !inside;
            }
        }

        return inside;
    }
}
