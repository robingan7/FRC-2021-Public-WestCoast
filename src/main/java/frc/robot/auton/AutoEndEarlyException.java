package frc.robot.auton;

public class AutoEndEarlyException extends Exception{
    private static final long serialVersionUID =1;//no idea what that means 
    public AutoEndEarlyException(){
        super("Auto End Early");
    }
}