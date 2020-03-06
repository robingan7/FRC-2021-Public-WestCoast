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

    @Override
    public boolean isHomeTurret() {
        return mOperatorJoystick.getRawButton(7);
    }

    @Override
    public boolean isAutoSteering() {
        return mOperatorJoystick.getRawButton(6);
    }

    @Override
    public boolean isTurretMoveRight() {
        return mOperatorJoystick.getPOV(0) == 90;
    }

    @Override
    public boolean isTurretMoveLeft() {
        return mOperatorJoystick.getPOV(0) == 270;
    }

    @Override
    public double getElevator() {
        return mOperatorJoystick.getRawAxis(5);
    }

    @Override
    public double getIntake() {
        return mOperatorJoystick.getRawAxis(1);
    }

    @Override
    public boolean isReversePasser() {
        return mOperatorJoystick.getRawButton(6);
    }

    public boolean isAutoAimming() {
        return mOperatorJoystick.getRawButton(6);//3 for xbox
    }
}