#include "posterior.h"

#include "parameters.h"

namespace virtual_scan
{


Collector::Collector(bool hits):
	hits(hits)
{
	// Nothing to do.
}


void Collector::operator() (bool mode, const Point2D &point)
{
	if (mode == hits)
		points.push_back(point);
}


void Collector::move(Reading &Z_i, Reading &Z_j)
{
	for (int i = 0, n = points.size(); i < n; i++)
	{
		const Point2D &point = points[i];
		Z_i.erase(point);
		Z_j.insert(point);
	}
}


Posterior::Posterior():
	S_len(0.0),
	S_mot(0.0),
	S_ms1(0.0),
	S_ms2(0.0),
	S_ms3(0.0),
	S_ms4(0.0)
{
	// Nothing to do.
}


double Posterior::operator () () const
{
	return exp(
		LAMBDA_L * S_len -
		LAMBDA_T * S_mot -
		LAMBDA_1 * S_ms1 -
		LAMBDA_2 * S_ms2 -
		LAMBDA_3 * S_ms3 -
		LAMBDA_4 * S_ms4
	);
}


void Posterior::update_S_len(int i, const Track::S &tracks)
{
	Track::ID id = tracks[i]->id;
	if (lengths.count(id) > 0)
		S_len -= lengths[id];

	lengths[id] = tracks[i]->size();
	S_len += lengths[id];
}


void Posterior::update_S_mot(int i, const Track::S &tracks)
{
}


void Posterior::update_S_ms1(const ObstaclePose &pose, const Reading &Z_k, bool ranged, Collector &collect)
{
	if (Z_k.size() == 0)
		return;

	ObstacleView view(pose);

	Reading::const_iterator i = Z_k.begin();
	Reading::const_iterator n = Z_k.end();
	if (ranged)
	{
		i = Z_k.lower_bound(view.range.first);
		n = Z_k.upper_bound(view.range.second);
	}

	double d_sum = 0.0;
	for (; i != n; ++i)
	{
		const Point2D &point = *i;
		if (view < point)
		{
			collect(false, point);
			break;
		}
		else if (view > point)
		{
			collect(false, point);
			continue;
		}

		double d = view.distance(point);
		if (d > D_MAX)
		{
			collect(false, point);
			continue;
		}

		d_sum += d;
		collect(true, point);
	}

	const Node *node = pose.node;
	if (distances.count(node) > 0)
		S_ms1 -= distances[node];

	distances[node] = d_sum;
	S_ms1 += d_sum;
}


void Posterior::update_S_ms1(int i, int j, const Track::S &tracks)
{
	const Track &track = *tracks[i];
	const ObstaclePose pose = track[j];
	const Node *node = pose.node;

	Reading &Zs_ij = Zs[node->timestamp];
	Reading &Zd_ij = Zd[node];

	Collector hits(true);
	Collector misses(false);

	update_S_ms1(pose, Zs_ij, true, hits);
	update_S_ms1(pose, Zd_ij, false, misses);

	hits.move(Zs_ij, Zd_ij);
	misses.move(Zd_ij, Zs_ij);
}


void Posterior::update_S_ms2(int i, int j, const Track::S &tracks)
{
}


void Posterior::update_S_ms3(int i, int j, const Track::S &tracks)
{
}


void Posterior::update_S_ms4(int i, int j, const Track::S &tracks)
{
}


void Posterior::diffuse(int i, int j, const Track::S &tracks)
{
	update_S_mot(i, tracks);
	update_S_ms1(i, j, tracks);
	update_S_ms2(i, j, tracks);
	update_S_ms3(i, j, tracks);
	update_S_ms4(i, j, tracks);
}


void Posterior::swap(int i, int j, const Track::S &tracks)
{
	update_S_len(i, tracks);
	update_S_len(j, tracks);

	update_S_mot(i, tracks);
	update_S_mot(j, tracks);
}


void Posterior::update(const Track::S &tracks, const Readings &readings)
{
}


} // namespace virtual_scan
