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
public class Matrix 
{
    private double[][] matrix;
    private int row;
    private int colum;
    public Matrix(int row, int colum)
    {
        this.row = row;
        this.colum = colum;
        this.matrix = new double[this.row][this.colum];
    }
    public Matrix(int row, int colum, double[] arr)
    {
        this.row = row;
        this.colum = colum;
        this.matrix = new double[this.row][this.colum];
        for(int i = 0; i < arr.length; i++)
        {
            this.matrix[i][0] = arr[i];
        }
    }
    public Matrix(double[][] arr)
    {
        row = arr.length;
        colum = arr[0].length;
        matrix = new double[row][colum];
        for(int i = 0; i < arr.length; i++)
        {
            for(int j = 0; j < arr[i].length; j++)
            {
                this.matrix[i][j] = arr[i][j];
            }
        }
    }
    public Matrix getStateMatrix()
    {
        return (new Matrix(matrix));
    }
    public double get(int index)
    {
        return this.matrix[index][0];
    }
    public Matrix minus(Matrix other)
    {
        Matrix minus = new Matrix(row, colum, matrix[0]);
        for(int i = 0; i < matrix.length; i++)
        {
            for(int j = 0; j < matrix[i].length; j++)
            {
                minus.matrix[i][j] = matrix[i][j] - other.matrix[i][j];
            }
        }
        return minus;
    }
    public Matrix mult(Matrix other)
    {
        Matrix mult = new Matrix(row, other.colum);
        for(int i = 0; i < mult.row; i++)
        {
            for(int j = 0; j < mult.colum; j++)
            {
                for(int k = 0; k < colum; k++)
                {
                    mult.matrix[i][j] += matrix[i][k] * other.matrix[k][j];
                }
            }
        }
        return mult;
    }
}
