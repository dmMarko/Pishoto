package frc.LibPurple.swerve.trajectories;

import java.util.Objects;

import frc.LibPurple.math.Rotation2D;
import frc.LibPurple.math.Vector2D;



public final class PathLineSegment extends PathSegment {
    private final Vector2D delta;

    public PathLineSegment(Vector2D start, Vector2D end) {
        super(start, end);
        this.delta = end.subtract(start);
    }

    @Override
    public PathLineSegment[] subdivide() {
        Vector2D mid = getPositionAtPercentage(0.5);
        return new PathLineSegment[] {
                new PathLineSegment(getStart(), mid),
                new PathLineSegment(mid, getEnd())
        };
    }

    @Override
    public PathLineSegment mirror() {
        return new PathLineSegment(
                getStart().multiply(1.0, -1.0),
                getEnd().multiply(1.0, -1.0)
        );
    }

    @Override
    public double getCurvature() {
        return 0;
    }

    @Override
    public Vector2D getPositionAtPercentage(double percentage) {
        return getStart().add(delta.scale(percentage));
    }

    @Override
    public Rotation2D getHeadingAtPercentage(double percentage) {
        return delta.getAngle();
    }

    @Override
    public double getLength() {
        return delta.length;
    }

    @Override
    public boolean equals(Object o) {
        if (o instanceof PathLineSegment) {
            PathLineSegment other = (PathLineSegment) o;
            return getStart().equals(other.getStart()) && delta.equals(other.delta);
        }

        return false;
    }

    @Override
    public int hashCode() {
        return Objects.hash(getStart(), delta);
    }

    @Override
    public String toString() {
        return String.format("{start: %s, end: %s}", getStart(), getEnd());
    }
}