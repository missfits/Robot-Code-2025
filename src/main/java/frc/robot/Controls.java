package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.Constants.OperatorConstants;
import frc.robot.RobotContainer.JoystickVals;

public class Controls {

    private static SlewRateLimiter xSlewRateLimiter = new SlewRateLimiter(OperatorConstants.SLEW_RATE_LIMIT);
    private static SlewRateLimiter ySlewRateLimiter = new SlewRateLimiter(OperatorConstants.SLEW_RATE_LIMIT);

    
    public static JoystickVals adjustDrivetrainInputs(double x, double y, boolean slowmode, boolean limitSlewRate) {
        return applySlewRateLimiter(adjustSlowmode(inputShape(x, y), slowmode), limitSlewRate); 
    }

    public static JoystickVals applySlewRateLimiter(JoystickVals input, boolean limitSlewRate) {
        if (limitSlewRate) {
            return new JoystickVals(xSlewRateLimiter.calculate(input.x()), ySlewRateLimiter.calculate(input.y()));
        } else {
            // still update the slewRateLimiters so it doesn't get mad at a discontinuity :)
            xSlewRateLimiter.reset(input.x());
            ySlewRateLimiter.reset(input.y());
            return input;
        }
    }

    // square the input while maintaining direction 
    public static JoystickVals inputShape(double x, double y) {
        double hypot = Math.hypot(x, y);
        double deadbandedValue = MathUtil.applyDeadband(hypot, OperatorConstants.JOYSTICK_DEADBAND);
    
        double scaleFactor= hypot == 0 ? 0 : deadbandedValue * Math.abs(deadbandedValue) / hypot; // avoid division by 0 issues

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

    public static double applyDeadband(double input) {
        return MathUtil.applyDeadband(input, OperatorConstants.JOYSTICK_DEADBAND);
    }

}
