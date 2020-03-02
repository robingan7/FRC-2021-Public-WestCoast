package frc.lib.utility;

/**
 * Returns true once if and only if the value of newValue changes from false to true.
 */
public class LatchBoolean {
    private boolean lastVal_ = false;

    public boolean getBoolean(boolean possibleNewValue) {
        boolean result = false;
        if (possibleNewValue && !lastVal_) {
            result = true;
        }
        lastVal_ = possibleNewValue;
        return result;
    }
}