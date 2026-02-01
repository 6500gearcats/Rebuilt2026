package frc.robot;

import java.util.HashMap;

/**
 * Performs linear interpolation between calibration points.
 */
public class LinearInterpolate {

    /** Lookup table of calibration points keyed by distance. */
    HashMap<Double, Double> points;

    /**
     * Finds the two nearest points and linearly interpolates between them.
     *
     * @param dist input distance
     * @return interpolated value
     */
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

    /**
     * Linearly interpolates between two values.
     *
     * @param start  start value
     * @param end    end value
     * @param amount normalized amount in the range [0, 1]
     * @return interpolated value
     */
    public double interpolate(double start, double end, double amount) {
        return start + (end - start) * amount;
    }
}
