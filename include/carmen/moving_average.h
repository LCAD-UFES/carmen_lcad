/*
 * Versão C++ do código http://alias-i.com/lingpipe/src/com/aliasi/stats/OnlineNormalEstimator.java
 * Adaptada por: Alberto e Claudine para o carmen_lcad
 * Abaixo, o texto original do autor. O código C++ depois do texto é nossa versão C++ carmen_lcad.
 * Data: 02/09/2020
 *
 */

/**
 * An {@code OnlineNormalEstimator} provides an object that estimates
 * means, variances, and standard deviations for a stream of numbers
 * presented one at a time.  Given a set of samples {@code
 * x[0],...,x[N-1]}, the mean is defined by:
 *
 * <blockquote><pre>
 * mean(x) = (1/N) * <big><big>&Sigma;</big></big><sub>i &lt; N</sub> x[i]</pre></blockquote>
 *
 * The variance is defined as the average squared difference from the mean:
 *
 * <blockquote><pre>
 * var(x) = (1/N) * <big><big>&Sigma;</big></big><sub>i &lt; N</sub> (x[i] - mean(x))<sup>2</sup></pre></blockquote>
 *
 * and the standard deviation is the square root of variance:
 *
 * <blockquote><pre>
 * dev(x) = sqrt(var(x))</pre></blockquote>
 *
 * <p>By convention, the mean and variance of a zero-length sequence
 * of numbers are both returned as 0.0.
 *
 * <p>The above functions provide the maximum likelihood estimates of
 * the mean, variance and standard deviation for a normal distribution
 * generating the values.  That is, the estimated parameters are the
 * parameters which assign the observed data sequence the highest probability.
 *
 * <p>Unfortunately, the maximum likelihood variance and deviation
 * estimates are biased in that they tend to underestimate variance in
 * general.  The unbiased estimates adjust counts downward by one, thus
 * adjusting variance and deviation upwards:
 *
 * <blockquote><pre>
 * varUnbiased(x) = (N / (N-1)) * var(x)
 * devUnbiased(x) = sqrt(varUnbiased(x))</pre></blockquote>
 *
 * Note that {@code var'(x) >= var(x)} and {@code dev'(x) >= dev(x)}.
 *
 *
 * <p><b>Welford's Algorithm</b>
 *
 * <p>This class use's Welford's algorithm for estimation.  This
 * algorithm is far more numerically stable than either using two
 * passes calculating sums, and sum of square differences, or using a
 * single pass accumulating the sufficient statistics, which are the
 * two moments, the sum, and sum of squares of all entries.  The
 * algorithm keeps member variables in the class, and performs the
 * following update when seeing a new variable {@code x}:
 *
 * <blockquote><pre>
 * long n = 0;
 * double mu = 0.0;
 * double sq = 0.0;
 *
 * void update(double x) {
 *     ++n;
 *     double muNew = mu + (x - mu)/n;
 *     sq += (x - mu) * (x - muNew)
 *     mu = muNew;
 * }
 * double mean() { return mu; }
 * double var() { return n > 1 ? sq/n : 0.0; }</pre></blockquote>
 *
 * <p><b>Welford's Algorithm with Deletes</b></p>
 *
 * LingPipe extends the Welford's algorithm to support deletes by
 * value.  Given current values of {@code n}, {@code mu}, {@code sq},
 * and any {@code x} added at some point, we can compute the previous
 * values of {@code n}, {@code mu}, {@code sq}.  The delete method is:
 *
 * <blockquote><pre>
 * void delete(double x) {
 *     if (n == 0) throw new IllegalStateException();
 *     if (n == 1) {
 *         n = 0; mu = 0.0; sq = 0.0;
 *         return;
 *     }
 *     muOld = (n * mu - x)/(n-1);
 *     sq -= (x - mu) * (x - muOld);
 *     mu = muOld;
 *     --n;
 * }</pre></blockquote>
 *
 * Because the data are exchangable for mean and variance
 * calculations (that is, permutations of the inputs produce
 * the same mean and variance), the order of removal does not
 * need to match the order of addition.
 *
 * <p><b>References</b></p>
 *
 * <ul>
 *
 * <li>Knuth, Donald E. (1998) <i>The Art of Computer Programming,
 * Volume 2: Seminumerical Algorithms, 3rd Edition.</i> Boston:
 * Addison-Wesley. Page 232.</li>
 *
 * <li>Welford, B. P. (1962) Note on a method for calculating
 * corrected sums of squares and products. <i>Technometrics</i>
 * <b>4</b>(3):419--420.</li>
 *
 * <li>Cook, John D. <a
 * href="http://www.johndcook.com/standard_deviation.html">Accurately
 * computing running variance</a>.</li>
 *
 *  </ul>
 *
 * @author  Bob Carpenter
 * @version 3.8.1
 * @since   Lingpipe3.8
 */

#include <string>
#include <carmen/carmen.h>

class carmen_moving_average_c
{

private:
	int mN = 0;
	double mM = 0.0;
	double mS = 0.0;

	/**
	 * Construct an instance of an online normal estimator that has
	 * seen no data.
	 */
public:
	carmen_moving_average_c()
	{
		/* intentionally blank */
	}

	/**
	 * Add the specified value to the collection of samples for this
	 * estimator.
	 *
	 * @param x Value to add.
	 */
	void add_sample(double x)
	{
		++mN;
		double nextM = mM + (x - mM) / mN;
		mS += (x - mM) * (x - nextM);
		mM = nextM;
	}

	/**
	 * Removes the specified value from the sample set.  See the class
	 * documentation above for the algorithm.
	 *
	 * @param x Value to remove from sample.
	 * @throws IllegalStateException If the current number of samples
	 * is 0.
	 */
	void remove_sample(double x)
	{
		if (mN == 0)
		{
			carmen_die("Trying to remove an element from an empty online average.");
		}
		if (mN == 1)
		{
			mN = 0;
			mM = 0.0;
			mS = 0.0;
			return;
		}
		double mOld = (mN * mM - x) / (mN - 1);
		mS -= (x - mM) * (x - mOld);
		mS = mS < 0.0 ? 0.0: mS; // para evitar negativar em casos de valores muito pequenos após a linha acima
		mM = mOld;
		--mN;
	}

	/**
	 * Returns the number of samples seen by this estimator.
	 *
	 * @return The number of samples seen by this estimator.
	 */
	int num_samples()
	{
		return mN;
	}

	/**
	 * Returns the arithmetic mean of the samples.
	 *
	 * @return The arithmetic mean of the samples.
	 */
	double arithmetic_mean()
	{
		return mM;
	}

	/**
	 * Returns the maximum likelihood estimate of the variance of
	 * the samples.
	 *
	 * @return Maximum likelihood variance estimate.
	 */
	double variance()
	{
		return mN > 1 ? mS / mN : 0.0;
	}

	/**
	 * Returns the unbiased estimate of the variance of the samples.
	 *
	 * @return Unbiased variance estimate.
	 */
	double variance_unbiased()
	{
		return mN > 1 ? mS / (mN - 1) : 0.0;
	}

	/**
	 * Returns the maximum likelihood estimate of the standard deviation of
	 * the samples.
	 *
	 * @return Maximum likelihood standard deviation estimate.
	 */
	double standard_deviation()
	{
		return sqrt(variance());
	}

	/**
	 * Returns the unbiased estimate of the standard deviation of the samples.
	 *
	 * @return Unbiased standard deviation estimate.
	 */
	double standard_deviation_unbiased()
	{
		return sqrt(variance_unbiased());
	}

	/**
	 * Clear statistics.
	 */
	void clear()
	{
		mN = 0;
		mM = 0.0;
		mS = 0.0;
	}
};
