package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CircularZone implements Zone {
    private final Translation2d center;
    private final double radius;

    public CircularZone(Translation2d center, double radius) {
        this.center = center;
        this.radius = radius;
    }

    @Override // This will now resolve correctly
    public Trigger contains(Supplier<Translation2d> translationSupplier) {
        return new Trigger(() -> {
            Translation2d current = translationSupplier.get();
            return current.getDistance(center) <= radius;
        });
    }
}
