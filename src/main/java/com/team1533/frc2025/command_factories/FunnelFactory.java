package com.team1533.frc2025.command_factories;

import java.util.function.DoubleSupplier;

import com.team1533.frc2025.RobotContainer;
import com.team1533.frc2025.subsystems.arm.ArmConstants;
import com.team1533.frc2025.subsystems.arm.ArmSubsystem;
import com.team1533.frc2025.subsystems.funnel.FunnelConstants;
import com.team1533.frc2025.subsystems.funnel.FunnelSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

public class FunnelFactory {
       private static final FunnelSubsystem funnel = RobotContainer.getInstance().getFunnelSubsystem();

    public static Command moveFunnelMotionMagic(DoubleSupplier setpoint) {
        return funnel.motionMagicSetpointCommand(setpoint).until(() -> MathUtil.isNear(setpoint.getAsDouble(), funnel.getCurrentPosition(), FunnelConstants.toleranceRotations));
    }

    public static Command moveArmDutyCycle(DoubleSupplier setpoint) {
        return funnel.dutyCycleCommand(setpoint);
    }

    public static Command hold() {
        return funnel.holdSetpointCommand();
    }
}
