package frc.utils;

public class EncoderConversions {
    public static double getEncoderValueFromDistance(double distance, double distancePerTick, double baseDistance) {
        return (distance - baseDistance) * (1.0 / distancePerTick);
    }

    public static double getEncoderValueFromDistance(double distance, double distancePerTick) {
        return getEncoderValueFromDistance(distance, distancePerTick, 0.0);
    }

    public static double getDistanceFromEncoderValue(double encoderValue, double distancePerTick, double baseDistance) {
        return encoderValue * distancePerTick + baseDistance;
    }

    public static double getDistanceFromEncoderValue(double encoderValue, double distancePerTick) {
        return getDistanceFromEncoderValue(encoderValue, distancePerTick, 0.0);
    }

    public static double getVelocity(double encoderValueChange, double distancePerTick, double deltaTime) {
        return (encoderValueChange * distancePerTick) / deltaTime;
    }
}
