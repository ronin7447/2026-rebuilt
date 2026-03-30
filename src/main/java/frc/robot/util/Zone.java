package frc.robot.util;

import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface Zone {
    /**
     * Returns a Trigger that is active when the provided translation is within the zone.
     */
    Trigger contains(Supplier<Translation2d> translation);
}

