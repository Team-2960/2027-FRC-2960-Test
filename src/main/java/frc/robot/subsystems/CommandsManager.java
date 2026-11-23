package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;

public class CommandsManager {

    private final CommandSwerveDrivetrain drivetrain;

    /**
     * Constructor
     * 
     * @param indexer      indexer subsystem instance
     * @param shooterWheel shooter wheel subsystem instance
     * @param shooterHood  shooter hood subsystem instance
     */
    public CommandsManager(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }
}