package frc.lib.utility;

public class DelayBoolean {
    protected double lastTimeStamp;
    protected final double kDelaySecond = 0.2;
    public DelayBoolean() {
        lastTimeStamp = 0;
    }

    public boolean canBeActived(boolean input, double timeStamp) {

        if(input && Math.abs(timeStamp - lastTimeStamp) > (Utility.epsilon + kDelaySecond)) {
            lastTimeStamp = timeStamp;
            return true;
        } else {
            return false;
        }
    }
}