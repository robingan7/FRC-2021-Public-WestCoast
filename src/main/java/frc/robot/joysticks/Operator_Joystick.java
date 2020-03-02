package frc.robot.joysticks;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Joystick;
public class Operator_Joystick implements IOperator_Joystick{
    private static final Operator_Joystick mInstance = new Operator_Joystick();

    public static Operator_Joystick getInstance(){
        return mInstance;
    }

    private Joystick mOperatorJoystick;

    private Operator_Joystick(){
        mOperatorJoystick = new Joystick(Constants.kOperatorJoystickId);
    }

    @Override
    public boolean isActivePanelControl() {
        return mOperatorJoystick.getRawButton(1);
    }

    @Override
    public boolean isKillPanelControl() {
        return mOperatorJoystick.getRawButton(2);
    }
}