package frc.lib.motor;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.ErrorCode;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DriverStation;


public class MotorUtility{
    private final static int kTimeoutMs = 100;

    public static class Configuration {
    }

    public static LazyTalonSRX createTalon(int id) {
        LazyTalonSRX talon = new LazyTalonSRX(id);
        //talon.set(ControlMode.PercentOutput, 0.0);
        
        return talon;
    }

    public static VictorSPX createVictor(int id, int master_id){
        VictorSPX victor = new VictorSPX(id);
        //victor.set(ControlMode.Follower, master_id);

        return victor;
    }

    public static VictorSPX createVictor(int id){
        VictorSPX victor = new VictorSPX(id);
        //victor.setNeutralMode(NeutralMode.Brake);

        return victor;
    }

    private class IllegalArgumentException extends Exception{
        private IllegalArgumentException(String error){
            super(error);
        }
    }

    public static Solenoid makeSolenoidFromId(int solenoidId) {
       
        return new Solenoid(solenoidId);
        
        //throw new IllegalArgumentException("Solenoid ID not valid: " + solenoidId);
    }

    public static void checkError(ErrorCode errorCode, String message) {
        if (errorCode != ErrorCode.OK) {
            DriverStation.reportError(message + errorCode, false);
        }
    }
}