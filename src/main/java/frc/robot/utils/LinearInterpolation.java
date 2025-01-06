package frc.robot.utils;

import java.util.List;

public class LinearInterpolation {
    private final List<Point> points;

    public LinearInterpolation(List<Point> points) {
        this.points = points;
    }

    public double calculate(double x) {

        if (x <= points.get(0).x) {
            return points.get(0).y;
        }
        for (int i = 0; i < points.size(); i++) {
            if (x <= points.get(i).x) {
                Point startPoint = points.get(i - 1);
                Point endPoint = points.get(i);

                return ((x - startPoint.x) / (endPoint.x - startPoint.x))
                        * (endPoint.y - startPoint.y)
                        + startPoint.y;
            }
        }
        return points.get(points.size() - 1).y;
    }

    public static class Point {
        public final double x;
        public final double y;

        public Point(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }
}