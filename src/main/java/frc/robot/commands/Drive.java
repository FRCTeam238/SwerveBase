package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

/**
 * Drive
 */
public class Drive extends CommandBase {

    Drivetrain drivetrain = Robot.drivetrain;

    OI oi = Robot.oi;
    public Drive() {}

    @Override
    public void initialize() {
        
        
    }

    @Override
    public void execute() {
        double leftJoyX = oi.leftJoystick.getX();
        double leftJoyY = oi.leftJoystick.getY();
        double rightJoyX = oi.rightJoystick.getX();

        drivetrain.drive(leftJoyX, leftJoyY, rightJoyX, true);
        
    }
    
}
