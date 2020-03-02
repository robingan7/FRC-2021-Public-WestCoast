package frc.lib.utility;

public class LatchDelayBoolean extends DelayBoolean {
    private boolean latchBoolean;
    public LatchDelayBoolean() {
        latchBoolean = false;
    }

    @Override
    public boolean canBeActived(boolean input, double timeStamp) {

        if(input && Math.abs(timeStamp - lastTimeStamp) > (Utility.epsilon + kDelaySecond)) {
            System.out.println("TTTTTIME: " +  timeStamp + " - " + lastTimeStamp + " input: " + input);

            lastTimeStamp = timeStamp;
            boolean returnVal = latchBoolean;
            latchBoolean = !returnVal;
            return !returnVal;
        } else {
            return !latchBoolean;
        }
    }
}