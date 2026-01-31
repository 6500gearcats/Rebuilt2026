package frc.robot;

import java.util.HashMap;

public class LinearInterpolate {

    HashMap<Double, Double> points;

    public double findAndInterpolate(double dist) {
        double lowerKey;
        double upperKey;

        if (dist < points.get(0)) {
            return points.get(0);
        }
        if (dist > points.get(points.size() - 1)) {
            return points.get(points.size() - 1);
        }
        lowerKey = points.keySet().stream().filter(k -> k <= dist).max(Double::compare).get();
        upperKey = points.keySet().stream().filter(k -> k >= dist).min(Double::compare).get();
        return interpolate(points.get(lowerKey), points.get(upperKey), (dist - lowerKey) / (upperKey - lowerKey));
    }

    public double interpolate(double start, double end, double amount) {
        return start + (end - start) * amount;
    }
}
