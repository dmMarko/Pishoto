package frc.LibPurple.swerve.Utils;

public interface InverseInterpolable<T> {
    double inverseInterpolate(T upper, T query);
}