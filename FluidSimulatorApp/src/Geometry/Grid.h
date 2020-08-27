#pragma once
#include <vector>
#include "Drawable.h"
#include <Eigen/Dense>
#include <limits>

struct GridCoordinates {
	int i;
	int j;

};
/*
Struct for arbitrary 2D grid... could also be implemented by an Eigen Matrix??
*/
template <class T>
struct Grid2D
{
	std::vector<T> data;

	int rows;
	int columns;

	/*
	Gets value at 2d index
	*/
	T getIndex(int i, int j)
	{
		return data[columns*i + j];
	}

	/*
	Sets value at 2D index
	*/
	void setIndex(int i, int j, T value)
	{
		data[columns*i + j] = value;
	}

	/*
	Increments value at index
	*/
	void incrementIndex(int i, int j, T increment)
	{
		data[columns*i + j] += increment;
	}

	/*
	Gets 2D position (x, y) with bottom left of window being 0, 0
	*/
	Eigen::Vector2f ind2pos(int i, int j, float dx)
	{
		Eigen::Vector2f pos;
		pos.x = (j * dx) + dx/2.0f;
		pos.y = (i * dx) + dx / 2.0f;
		return pos;
	}

	/*
	Gets 2D coordinates on grid (i, j) with bottom left of window being 0, 0
	*/
	GridCoordinates pos2ind(Eigen::Vector2f pos, float dx)
	{
		GridCoordinates coords;
		coords.i = (int) (pos.y() / dx);
		coords.j = (int) (pos.x() / dx);
		return coords;
	}

	/*
	Set value of all elements in data to zero
	*/
	void zero()
	{
		for (int i = 0; i < rows; i++)
		{
			for (int j = 0; j < columns; j++)
			{
				setIndex(i, j, 0.0f);
			}
		}
	}
	/*
	Goes through 2D array, checks if value is maximum allowed of type, if it is, set it to zero
	*/
	void handleUnknownValues()
	{
		for (int i = 0; i < rows; i++)
		{
			for (int j = 0; j < columns; j++)
			{
				if (getIndex(i, j) < -10000000 || getIndex(i, j) > 10000000 )
				{
					setIndex(i, j, 0.0f);
				}
			}
		}
	}

};


/*

using namespace Eigen;
struct ImageGrid2D : public Drawable
{

};
*/