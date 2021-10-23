package frc.LibPurple.swerve.trajectories;

import java.util.Objects;

import frc.LibPurple.math.RigidTransform2D;
import frc.LibPurple.math.Rotation2D;
import frc.LibPurple.math.Vector2D;


public final class PathArcSegment extends PathSegment {
    private final Vector2D center;
    private final Vector2D deltaStart;
    private final Vector2D deltaEnd;
    private final boolean clockwise;

    public PathArcSegment(Vector2D start, Vector2D end, Vector2D center) {
        super(start, end);
        this.center = center;
        this.deltaStart = start.subtract(center);
        this.deltaEnd = end.subtract(center);

        clockwise = deltaStart.cross(deltaEnd) <= 0.0;
    }

    /**
     * Constructs an arc from 3 points along an arc.
     *
     * @param a the point where the arc begins
     * @param b a point along the arc
     * @param c the point where the arc ends
     *
     * @return an arc going through the given points or null if an arc cannot be constructed out of the points
     */
    public static PathArcSegment fromPoints(Vector2D a, Vector2D b, Vector2D c) {
        PathLineSegment chordAB = new PathLineSegment(a, b);
        PathLineSegment chordBC = new PathLineSegment(b, c);

        RigidTransform2D perpChordAB = new RigidTransform2D(chordAB.getPositionAtPercentage(0.5), chordAB.getHeadingAtPercentage(0.5).normal());
        RigidTransform2D perpChordBC = new RigidTransform2D(chordBC.getPositionAtPercentage(0.5), chordBC.getHeadingAtPercentage(0.5).normal());

        Vector2D center = perpChordAB.intersection(perpChordBC);

        if (!Double.isFinite(center.x) || !Double.isFinite(center.y)) {
            return null;
        }


        return new PathArcSegment(a, c, center);
    }

    @Override
    public PathArcSegment[] subdivide() {
        Vector2D mid = getPositionAtPercentage(0.5);
        return new PathArcSegment[] {
                new PathArcSegment(getStart(), mid, center),
                new PathArcSegment(mid, getEnd(), center)
        };
    }

    @Override
    public PathArcSegment mirror() {
        return new PathArcSegment(
                getStart().multiply(1.0, -1.0),
                getEnd().multiply(1.0, -1.0),
                center.multiply(1.0, -1.0)
        );
    }

    @Override
    public double getCurvature() {
        return 1.0 / deltaStart.length;
    }

    @Override
    public Vector2D getPositionAtPercentage(double percentage) {
        double deltaAngle = Vector2D.getAngleBetween(deltaStart, deltaEnd).toRadians() *
                (clockwise ? -1.0 : 1.0) * percentage;
        return center.add(deltaStart.rotateBy(Rotation2D.fromRadians(deltaAngle)));
    }

    @Override
    public Rotation2D getHeadingAtPercentage(double percentage) {
        double angle = Vector2D.getAngleBetween(deltaStart, deltaEnd).toRadians() *
                (clockwise ? -1.0 : 1.0) * percentage +
                (clockwise ? -0.5 * Math.PI : 0.5 * Math.PI); // Add or subtract 90 degrees to the angle based on the direction of travel
        return deltaStart.rotateBy(Rotation2D.fromRadians(angle)).getAngle();
    }

    public Vector2D getCenter() {
        return center;
    }

    @Override
    public double getLength() {
        return deltaStart.length * Vector2D.getAngleBetween(deltaStart, deltaEnd).toRadians();
    }

    public double getRadius() {
        return deltaStart.length;
    }

    @Override
    public boolean equals(Object o) {
        if (o instanceof PathArcSegment) {
            PathArcSegment other = (PathArcSegment) o;
            return deltaStart.equals(other.deltaStart) && deltaEnd.equals(other.deltaEnd) && center.equals(other.center);
        }

        return false;
    }

    @Override
    public int hashCode() {
        return Objects.hash(center, deltaStart, deltaEnd);
    }

    @Override
    public String toString() {
        return String.format("{start: %s, end: %s, center: %s}", getStart(), getEnd(), center);
    }
}