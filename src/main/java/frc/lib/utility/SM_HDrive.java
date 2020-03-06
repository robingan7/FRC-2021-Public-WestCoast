package frc.lib.utility;

import frc.lib.utility.DriveSignal;

public class SM_HDrive {
    private static final double angleMin = 10;
    private static final double angleMax = 20;
    private static final double angleMaxHLimit = 85;

    public static double[] getHDriveOutput(double moveIn, double hdriveIn) {
        double angle = getAngle(moveIn, hdriveIn);

        //System.out.println("test: " + angle);
        if(Math.abs(angle) < angleMin) {
            return new double[]{0.0, mapFromUnitCircle(moveIn, hdriveIn)};
        } else if(Math.abs(angle) > angleMax) {
            return new double[]{mapFromUnitCircle(hdriveIn, moveIn), mapFromUnitCircle(moveIn, hdriveIn)};
        }

        if(Math.abs(angle) > angleMaxHLimit) {
            hdriveIn = 0;
        }
        return new double[]{moveIn, hdriveIn};
    }

    private static double getAngle(double y, double x) {
        return Math.toDegrees(Math.atan(y / x));
    }

    private static double mapFromUnitCircle(double y, double x) {
        double xMax = Math.sqrt(1 - y * y);
        return map(x, -xMax, xMax, -1, 1);
    }

    private static double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

}