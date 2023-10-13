package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.Drive;

/**
 * OI
 */
public class OI {

    public Joystick leftJoystick = new Joystick(0);
    public Joystick rightJoystick = new Joystick(0);
    
    public OI() {
        Robot.drivetrain.setDefaultCommand(new Drive());
    }
}
