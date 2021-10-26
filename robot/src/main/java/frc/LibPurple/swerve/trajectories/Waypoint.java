package frc.LibPurple.swerve.trajectories;

import frc.LibPurple.math.Rotation2D;
import frc.LibPurple.math.Vector2D;

public class Waypoint {
    public final Vector2D position;
    public final Rotation2D heading;
    public final Rotation2D rotation;

    public Waypoint(Vector2D position, Rotation2D heading) {
        this.position = position;
        this.heading = heading;
        this.rotation = heading;
    }

    public Waypoint(Vector2D position, Rotation2D heading, Rotation2D rotation) {
        this.position = position;
        this.heading = heading;
        this.rotation = rotation;
    }
}