package frc.robot;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.OperatorConstants;
import frc.robot.RobotContainer.JoystickVals;

public class Controls {
    
    public static JoystickVals adjustInputs(double x, double y, boolean slowmode) {
        return adjustSlowmode(inputShape(x, y), slowmode);
    }


    // square the input while maintaining direction 
    public static JoystickVals inputShape(double x, double y) {
        double hypot = Math.hypot(x, y);
        double deadbandedValue = MathUtil.applyDeadband(hypot, OperatorConstants.JOYSTICK_DEADBAND);
    
        double scaleFactor = deadbandedValue * Math.abs(deadbandedValue) / hypot;

        JoystickVals output = new JoystickVals(x * scaleFactor, y * scaleFactor);

        return output;

    }

    public static JoystickVals adjustSlowmode(JoystickVals input, boolean slowmode) {
        if (slowmode) {
            return new JoystickVals(input.x() * OperatorConstants.SLOWMODE_FACTOR, input.y() * OperatorConstants.SLOWMODE_FACTOR);
        } else {
            return new JoystickVals(input.x(), input.y());
        }
    }

}
