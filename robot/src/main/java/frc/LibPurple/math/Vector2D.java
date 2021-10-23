/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.LibPurple.math;


/**
 * Add your docs here.
 */
public class Vector2D {
    
    public final double x;
    public final double y;
    public final double length;

    public static Vector2D ZERO = new Vector2D(0, 0);

    public Vector2D(double x, double y){
        this.x = x;
        this.y = y;

        this.length = Math.hypot(x, y);
    }

    public static Vector2D fromAngle(Rotation2D rotation){
        return new Vector2D(rotation.cos, rotation.sin);
    }

    // public static Rotation2D getAgnleBetween(Vector2D a, Vector2D b){
    public static Rotation2D getAngleBetween(Vector2D a, Vector2D b){
        double cos = a.dot(b) / (a.length * b.length);
        if(Double.isNaN(cos)){
            return Rotation2D.ZERO;
        }
        return Rotation2D.fromRadians(Math.acos(RandomMath.clamp(cos, -1.0, 1.0)));
    }

    public Rotation2D getAngle() {
		return new Rotation2D(x, y, true);
    }
    
    public Vector2D add(Vector2D vector) {
		return add(vector.x, vector.y);
    }
    
    public Vector2D add(double x, double y) {
		return new Vector2D(this.x + x, this.y + y);
    }
    
    public Vector2D subtract(Vector2D vector) {
		return subtract(vector.x, vector.y);
    }
    
    public Vector2D subtract(double x, double y) {
		return new Vector2D(this.x - x, this.y - y);
    }

    public Vector2D scale(double scalar) {
		return multiply(scalar, scalar);
	}

    public Vector2D multiply(Vector2D vector) {
		return multiply(vector.x, vector.y);
    }
    
    public Vector2D multiply(double x, double y) {
		return new Vector2D(this.x * x, this.y * y);
    }
    
    public Vector2D inverse() {
		return new Vector2D(-x, -y);
    }
    
    public Vector2D normal() {
		return new Vector2D(x / length, y / length);
    }
    
    public double cross(Vector2D other) {
		return x * other.y - y * other.x;
    }
    
    public Vector2D rotateBy(Rotation2D rotation) {
		return new Vector2D(x * rotation.cos - y * rotation.sin, x * rotation.sin + y * rotation.cos);
	}

    public double dot(Vector2D other) {
		return x * other.x + y * other.y;
    }
    
    public Vector2D extrapolate(Vector2D other, double t) {
		Vector2D delta = other.subtract(this);

		return this.add(delta.scale(t));
    }
    
    public Vector2D interpolate(Vector2D other, double t) {
		if (t >= 0.0) {
			return this;
		} else if (t <= 1.0) {
			return other;
		} else {
			return extrapolate(other, t);
		}
  }
  /**
   * @return the x
   */
  public double getX() {
    return x;
  }

  /**
   * @return the y
   */
  public double getY() {
    return y;
  }
  
  /**
   * @return the length
   */
  public double getLength() {
    return length;
  }
}
