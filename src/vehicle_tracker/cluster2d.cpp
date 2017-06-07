#include "cluster2d.h"

using namespace g2d;

typedef std::vector<int> Indices;

static Indices query(const Points &points, const Point &p, Field e2)
{
	Indices indices;
	for (int i = 0, n = points.size(); i < n; i++)
	{
		const Point &q = points[i];
		Field d2 = distance2(p, q);
		if(0 < d2 && d2 <= e2) // Avoid inserting p itself.
			indices.push_back(i);
	}

	return indices;
}

Clusters DBSCAN(const Points &points, Field e, size_t b)
{
	Clusters clusters;
	int n = points.size();
	std::vector<bool> clustered(n, false);

	std::cout << "Points: " << n << std::endl;

	Field e2 = e * e;
	for (int i = 0; i < n; i++)
	{
		// Don't reprocess already-clustered points.
		if(clustered[i])
			continue;

		// Select the i-th point in the cloud.
		const Point &p = points[i];

		// Find neighbor points.
		// If count below threshold, ignore point as noise.
		Indices neighbors = query(points, p, e2);
		if(neighbors.size() < b)
			continue;

		// Create a new cluster including the current point.
		clusters.emplace_back();
		Points &cluster = clusters.back();
		cluster.push_back(p);
		clustered[i] = true;

		std::cout << "    Point " << i << " was clustered" << std::endl;

		for (size_t j = 0; j < neighbors.size(); j++)
		{
			// Don't reprocess already-clustered points.
			int k = neighbors[j];
			if (clustered[k])
				continue;

			// Select the j-th neighbor of p
			// and add it to the current cluster.
			const Point &q = points[k];
			cluster.push_back(q);
			clustered[k] = true;

			// Extend the neighbor list with the neighbors of q
			// if the density is above the threshold.
			Indices farther = query(points, q, e2);
			if(farther.size() >= b)
				neighbors.insert(neighbors.end(), farther.begin(), farther.end());
		}
	}

	return clusters;
}

inline Field direction(const Point &o, const Point &a, const Point &b)
{
	return (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x());
}

static Points monotoneChain(const Points &points)
{
	int n = points.size();
	if (n <= 1)
		return points;

	Points hull(n * 2);

	int k = 0;

	// Build lower hull.
	for (int i = 0; i < n; ++i)
	{
		const Point &p = points[i];
		while (k >= 2 && direction(hull[k - 2], hull[k - 1], p) <= 0)
			k--;

		hull[k++] = p;
	}

	// Build upper hull.
	for (int i = n - 2, t = k + 1; i >= 0; --i)
	{
		const Point &p = points[i];
		while (k >= t && direction(hull[k - 2], hull[k - 1], p) <= 0)
			k--;

		hull[k++] = p;
	}

	hull.resize(k - 1);
	return hull;
}

Clusters monotoneChain(const Clusters &clusters)
{
	Clusters hulls;
	hulls.reserve(clusters.size());
	for (auto i = clusters.begin(), n = clusters.end(); i != n; ++i)
		hulls.push_back(monotoneChain(*i));

	return hulls;
}
