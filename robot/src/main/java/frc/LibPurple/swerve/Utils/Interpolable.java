package frc.LibPurple.swerve.Utils;

public interface Interpolable<T> {
    T interpolate(T other, double t);
}