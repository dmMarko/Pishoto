/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.LibPurple.swerve.trajectories;

import java.util.ArrayList;
import java.util.List;

import frc.LibPurple.swerve.Utils.InterpolatingDouble;
import frc.LibPurple.swerve.Utils.InterpolatingTreeMap;
import frc.LibPurple.swerve.Utils.Utils;
import frc.LibPurple.math.Rotation2D;
import frc.LibPurple.math.Vector2D;

/**
 * Add your docs here.
 */
public class Path {
    private final List<PathSegment> segments = new ArrayList<PathSegment>();
    private final List<Double> distancesFromStart = new ArrayList<Double>();

    private final InterpolatingTreeMap<InterpolatingDouble, Rotation2D> rotationAtDistance = new InterpolatingTreeMap<>();

    private double length = 0.0;

    public Path(Rotation2D startRotation)
    {
        rotationAtDistance.put(new InterpolatingDouble(0.0), startRotation);
    }

    public void addSegment(PathSegment segment)
    {
        segments.add(segment);
        distancesFromStart.add(length);
        length += segment.getLength();
    }

    public List<PathSegment> getSegments()
    {
        return segments;
    }

    public void addSegment(PathSegment segment, Rotation2D endRotation) {
        if(segment instanceof PathArcSegment){
            Utils.printErr("arc");
        }
        if(segment instanceof PathLineSegment){
            Utils.printErr("line");
        }
        addSegment(segment);
        rotationAtDistance.put(new InterpolatingDouble(length), endRotation);
    }

    public double getDistanceToSegmentStart(int segment)
    {
        return distancesFromStart.get(segment);
    }

    public double getDistanceToSegmentEnd(int segment) {
        return distancesFromStart.get(segment) + segments.get(segment).getLength();
    }

    private int getSegmentAtDistance(double distance) {
        int start = 0;
        int end = segments.size() - 1;
        int mid = start + (end - start) / 2;

        while (start < end) {
            mid = start + (end - start) / 2;

            if (distance > getDistanceToSegmentEnd(mid)) {
                start = mid + 1;
            } else if (distance < getDistanceToSegmentStart(mid)) {
                end = mid;
            } else {
                break;
            }
        }

        return mid;
    }

    public Vector2D getPositionAtDistance(double distance) {
        int currentSegment = getSegmentAtDistance(distance);
        double segmentDistance = distance - getDistanceToSegmentStart(currentSegment);

        return segments.get(currentSegment).getPositionAtDistance(segmentDistance);
    }

    public Rotation2D getHeadingAtDistance(double distance) {
        int currentSegment = getSegmentAtDistance(distance);
        double segmentDistance = distance - getDistanceToSegmentStart(currentSegment);

        return segments.get(currentSegment).getHeadingAtDistance(segmentDistance);
    }

    public Rotation2D getRotationAtDistance(double distance) {
        return rotationAtDistance.getInterpolated(new InterpolatingDouble(distance));
    }

    public double getLength()
    {
        return length;
    }
}
