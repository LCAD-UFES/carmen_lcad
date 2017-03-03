#include "munkres.h"

#include <boost/function.hpp>

#include <set>
#include <stdexcept>


namespace udatmo
{


// Entry states.
const int STAR = 1;
static const int NORMAL = 0;
static const int PRIME = 2;


/**
 * @brief State data used by the Munkres algorithm.
 */
struct Munkres
{
	cv::Mat_<bool> row_mask;

	cv::Mat_<bool> col_mask;

	cv::Mat mask_matrix;

	cv::Mat matrix;

	int saverow, savecol;

	Munkres(cv::Mat &data, cv::Mat &assignments);
};


Munkres::Munkres(cv::Mat &data, cv::Mat &assignments):
	row_mask(1, data.rows, false),
	col_mask(1, data.rows, false),
	mask_matrix(assignments),
	matrix(data)
{
	// Nothing to do.
}


/**
 * @brief A step in the Munkres algorithm.
 *
 * This is a solution to the function-that-returns-its-own-type as described here:
 * http://stackoverflow.com/questions/12304438/how-can-i-get-a-boostfunction-or-other-generic-function-wrapper-that-returns
 */
struct Step: boost::function<Step(Munkres&)>
{
    // A convenient alias for the base type.
    typedef boost::function<Step(Munkres&)> functor;

    /**
	 * @brief Default constructor.
     */
    Step(): functor()
	{
        // Nothing to do.
    }

    /**
	 * @brief Wraps a custom function into a Step object.
     */
    template<class F> Step(F f) : functor(f)
	{
        // Nothing to do.
    }
};


/**
 * @brief Signal the end of the algorithm.
 */
Step step0(Munkres &/*state*/)
{
	return step0;
}


Step step1(Munkres &state);


Step step2(Munkres &state);


Step step3(Munkres &state);


Step step4(Munkres &state);


Step step5(Munkres &state);


Step step6(Munkres &state);


inline double minimum(const cv::Mat &data)
{
	double minVal, maxVal;
	cv::minMaxLoc(data, &minVal, &maxVal);
	return minVal;
}


bool find_uncovered_in_matrix(Munkres &state, double item, int &row, int &col)
{
	cv::Mat matrix = state.matrix;
	int side = matrix.rows;

	for (row = 0; row < side; row++ )
	{
		if (state.row_mask(row))
			continue;

		for (col = 0; col < side; col++)
		{
			if (state.col_mask(col))
				continue;

			if (matrix.at<double>(row, col) == item)
				return true;
		}
	}

	return false;
}


Step step1(Munkres &state)
{
	cv::Mat matrix = state.matrix;
	cv::Mat mask_matrix = state.mask_matrix;

	int size = matrix.rows;
	for (int row = 0; row < size; row++)
	{
		for (int col = 0; col < size; col++)
		{
			if (matrix.at<double>(row, col) != 0)
				continue;

			bool starred = false;
			for (int row2 = 0; row2 < size; row2++)
			{
				if (mask_matrix.at<int>(row2, col) == STAR)
				{
					starred = true;
					break;
				}
			}

			if (starred)
				continue;

			for (int col2 = 0; col2 < size; col2++)
			{
				if (mask_matrix.at<int>(row, col2) == STAR)
				{
					starred = true;
					break;
				}
			}

			if (!starred)
			{
				mask_matrix.at<int>(row, col) = STAR;
			}
		}
	}

  return step2;
}


Step step2(Munkres &state)
{
	cv::Mat matrix = state.matrix;
	cv::Mat mask_matrix = state.mask_matrix;
	int size = matrix.rows;
	int covercount = 0;

	for (int row = 0; row < size; row++)
	{
		for (int col = 0; col < size; col++)
		{
			if (mask_matrix.at<int>(row, col) == STAR)
			{
				state.col_mask(col) = true;
				covercount++;
			}
		}

		if (covercount >= size)
			return step0;
	}

	return step3;
}


Step step3(Munkres &state)
{
	// Main Zero Search
	//
	// 1. Find an uncovered Z in the distance matrix and prime it. If no such zero exists, go to Step 5
	// 2. If No Z* exists in the row of the Z', go to Step 4.
	// 3. If a Z* exists, cover this row and uncover the column of the Z*. Return to Step 3.1 to find a new Z

	cv::Mat mask_matrix = state.mask_matrix;
	if (find_uncovered_in_matrix(state, 0, state.saverow, state.savecol))
		mask_matrix.at<int>(state.saverow, state.savecol) = PRIME;
	else
		return step5;

	cv::Mat matrix = state.matrix;
	for (int col = 0, size = matrix.rows; col < size; col++)
	{
		if (mask_matrix.at<int>(state.saverow, col) == STAR)
		{
			state.row_mask(state.saverow) = true; //cover this row and
			state.col_mask(col) = false; // uncover the column containing the starred zero
			return step3; // repeat
		}
	}

	return step4; // no starred zero in the row containing this primed zero
}


struct PointComparator
{
	bool operator () (const cv::Point &a, const cv::Point &b) const
	{
		if (a.x < b.x)
			return true;

		if (a.x > b.x)
			return false;

		return (a.y <= b.y);
	}
};


typedef std::set<cv::Point, PointComparator> Points;


Step step4(Munkres &state)
{
	// seq contains pairs of row/column values where we have found
	// either a star or a prime that is part of the ``alternating sequence``.
	Points seq;

	// use saverow, savecol from step 3.
	cv::Point z0(state.saverow, state.savecol);
	seq.insert(z0);

	// We have to find these two pairs:
	cv::Point z1(-1, -1);
	cv::Point z2n(-1, -1);

	/*
	Increment Set of Starred Zeros

	1. Construct the ``alternating sequence'' of primed and starred zeros:

	Z0 : Unpaired Z' from Step 4.2
	Z1 : The Z* in the column of Z0
	Z[2N] : The Z' in the row of Z[2N-1], if such a zero exists
	Z[2N+1] : The Z* in the column of Z[2N]

	The sequence eventually terminates with an unpaired Z' = Z[2N] for some N.
	*/

	cv::Mat matrix = state.matrix;
	cv::Mat mask_matrix = state.mask_matrix;

	int row = 0;
	int col = state.savecol;
	int size = matrix.rows;
	bool madepair = false;
	do
	{
		madepair = false;
		for (row = 0 ; row < size; row++ ) {
			if (mask_matrix.at<int>(row, col) == STAR)
			{
				z1 = cv::Point(row, col);
				if (seq.count(z1) > 0)
					continue;

				madepair = true;
				seq.insert(z1);
				break;
			}
		}

		if (!madepair)
			break;

		madepair = false;

		for (col = 0; col < size; col++)
		{
			if (mask_matrix.at<int>(row, col) == PRIME)
			{
				z2n = cv::Point(row, col);
				if (seq.count(z2n) > 0)
					continue;

				madepair = true;
				seq.insert(z2n);
				break;
			}
		}
	}
	while (madepair);

	for (Points::iterator i = seq.begin(), n = seq.end(); i != n; ++i)
	{
		const cv::Point &p = *i;

		// 2. Unstar each starred zero of the sequence.
		if (mask_matrix.at<int>(p.x, p.y) == STAR)
			mask_matrix.at<int>(p.x, p.y) = NORMAL;

		// 3. Star each primed zero of the sequence,
		// thus increasing the number of starred zeros by one.
		if (mask_matrix.at<int>(p.x, p.y) == PRIME)
			mask_matrix.at<int>(p.x, p.y) = STAR;
	}

	// 4. Erase all primes, uncover all columns and rows,
	for (row = 0; row < size; row++)
		for (col = 0 ; col < size; col++)
			if (mask_matrix.at<int>(row, col) == PRIME)
				mask_matrix.at<int>(row, col) = NORMAL;

	state.row_mask = false;
	state.col_mask = false;

	// and return to Step 2.
	return step2;
}


Step step5(Munkres &state)
{
	/*
	New Zero Manufactures

	1. Let h be the smallest uncovered entry in the (modified) distance matrix.
	2. Add h to all covered rows.
	3. Subtract h from all uncovered columns
	4. Return to Step 3, without altering stars, primes, or covers.
	*/

	double h = 0;
	cv::Mat matrix = state.matrix;
	int size = matrix.rows;
	for (int row = 0; row < size; row++)
	{
		if (state.row_mask(row))
			continue;

		for (int col = 0; col < size; col++) {
			if (state.col_mask(col))
				continue;

			if ((h > matrix.at<double>(row, col) && matrix.at<double>(row, col) != 0) ||
				h == 0)
				h = matrix.at<double>(row, col);
		}
	}

	for (int row = 0; row < size; row++) {
		if (!state.row_mask(row))
			continue;

		for (int col = 0 ; col < size; col++)
			matrix.at<double>(row, col) += h;
	}

	for (int col = 0; col < size; col++)
	{
		if (state.col_mask(col))
			continue;

		for (int row = 0; row < size; row++)
			matrix.at<double>(row, col) -= h;
	}

	return step3;
}


inline void minimize_along_direction(cv::Mat &matrix, bool over_columns)
{
	// Look for a minimum value to subtract from all values along
	// the "outer" direction.
	for (int i = 0, n = matrix.rows; i < n; i++)
	{
		cv::Mat streak = (over_columns ? matrix.col(i) : matrix.row(i));
		double min = minimum(streak);
		if (min > 0)
			streak -= min;
	}
}


/*
*
* Linear assignment problem solution
* [modifies matrix in-place.]
*
* Assignments are STAR values in the mask matrix.
*/
cv::Mat munkres(const cv::Mat &data, double roof)
{
	size_t rows = data.rows;
	size_t cols = data.cols;

	cv::Mat costs;
	if (rows == cols)
		costs = data.clone();
	else
	{
		int side = std::max(data.rows, data.cols);
		costs = cv::Mat(side, side, CV_64F, cv::Scalar(roof));
		cv::Mat roi(costs, cv::Rect(0, 0, cols, rows));
		data.copyTo(roi);
	}

	// Prepare the matrix values...
	minimize_along_direction(costs, false);
	minimize_along_direction(costs, true);

	cv::Mat assignments(costs.size(), CV_32S, cv::Scalar(NORMAL));
	Munkres state(costs, assignments);
	Step step = step1;
	while (step != step0)
		step = step(state);

	return assignments;
}


} //namespace udatmo
