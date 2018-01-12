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
	S_ms3(0.0)
{
	// Nothing to do.
}


double Posterior::operator () () const
{
	return exp(
		LAMBDA_L * S_len -
		LAMBDA_T * S_mot -
		LAMBDA_1 * S_ms1 -
		LAMBDA_3 * S_ms3
	);
}


void Posterior::erase_node(bool keep_sensor_points, int i, int j, const Track::S &tracks)
{
	Node *node = tracks[i][j].node;
	node->deselect();

	S_ms1 -= distances[node];
	distances.erase(node);

	if (keep_sensor_points)
	{
		Reading &Zd_i = Zd[node];
		update_S_ms3(tracks, Zd_i);
		Zs[node->timestamp].merge(Zd_i);
	}

	Zd.erase(node);
}


void Posterior::update_S_len(const Track &track)
{
	Track::ID id = track.id;
	if (lengths.count(id) > 0)
		S_len -= lengths[id];

	lengths[id] = track.size();
	S_len += lengths[id];
}


void Posterior::update_S_len(int i, const Track::S &tracks)
{
	update_S_len(tracks[i]);
}


void Posterior::update_S_mot(const Track &track)
{
	update_S_mot(0, track.size(), track);
}


void Posterior::update_S_mot(int i, const Track::S &tracks)
{
	update_S_mot(tracks[i]);
}


void Posterior::update_S_mot(int a, int n, const Track &track)
{
	for (int i = a; i < n; i++)
	{
		const ObstaclePose &pose = track[i];
	}
}


void Posterior::update_S_ms1(const ObstaclePose &pose, const Reading &Z_k, bool ranged, Collector &collect)
{
	if (Z_k.size() == 0)
		return;

	ObstacleView view(pose);

	const_iterator_chain<Reading> i, n;
	if (ranged)
	{
		i = Z_k.lower_bound(view.range);
		n = Z_k.upper_bound(view.range);
	}
	else
	{
		std::pair<double, double> all = std::make_pair(0, M_PI);
		i = Z_k.lower_bound(all);
		n = Z_k.upper_bound(all);
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


void Posterior::update_S_ms1(int j, const Track &track)
{
	const ObstaclePose &pose = track[j];
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


void Posterior::update_S_ms1(int i, int j, const Track::S &tracks)
{
	update_S_ms1(j, tracks[i]);
}


void Posterior::update_S_ms3(int j, const Track &track)
{
}


void Posterior::update_S_ms3(int i, int j, const Track::S &tracks)
{
}


void Posterior::update_S_ms3(const Track::S &tracks, const Reading &Z)
{
	const Pose &origin = Z.origin;
	for (int i = 0, m = tracks.size(); i < m; i++)
	{
		const Track &track = tracks[i];
		for (int j = 0, n = track.size(); j < n; j++)
		{
			const ObstaclePose &pose = track[j];
			const Node *node = pose.node;
			double l_2 = 0.5 * node->model->length; // The model rectangle is
			double w_2 = 0.5 * node->model->width;  // "lying" on its side.

			ObstacleView view(origin, pose);

			int count = 0;
			for (auto point = Z.lower_bound(view.range), done = Z.upper_bound(view.range); point != done; ++point)
			{
				PointXY local = pose.project_local(origin, *point);
				if (std::abs(local.x) < l_2 && std::abs(local.y) < w_2)
					count++;
			}

			if (collisions.count(node) > 0)
				S_ms3 -= collisions[node];

			collisions[node] = count;
			S_ms3 += count;
		}
	}
}


void Posterior::create(int i, const Track::S &tracks)
{
	update_S_len(i, tracks);
	update_S_mot(i, tracks);

	for (int j = 0, n = tracks[i].size(); j < n; j++)
	{
		update_S_ms1(i, j, tracks);
		update_S_ms3(i, j, tracks);
	}
}


void Posterior::destroy(int i, const Track::S &tracks)
{
	const Track &track = tracks[i];
	Track::ID id = track.id;

	// Update the length term and the length map.
	S_len -= lengths[id];
	lengths.erase(id);

	// Update the consistency term and the consistency map.
	S_mot -= covariances[id];
	covariances.erase(id);

	// Reassign sensor points related to the removed poses
	// as static readings.
	for (int j = 0, n = track.size(); j < n; j++)
		erase_node(true, i, j, tracks);
}


void Posterior::extend_forward(const Track &track)
{
	Track::ID id = track.id;
	int j = track.size() - 1;

	// Update the length term and the length map.
	S_len++;
	lengths[id]++;

	update_S_mot(track);
	update_S_ms1(j, track);
	update_S_ms3(j, track);
}


void Posterior::extend_backward(const Track &track)
{
	Track::ID id = track.id;

	// Update the length term and the length map.
	S_len++;
	lengths[id]++;

	update_S_mot(track);
	update_S_ms1(0, track);
	update_S_ms3(0, track);
}


void Posterior::pop_back(int i, int k, const Track::S &tracks)
{
	const Track &track = tracks[i];
	Track::ID id = track.id;

	// Update the length map and the length term.
	int n = tracks.size();
	int w = n - k;
	lengths[id] -= w;
	S_len -= w;

	// Update the consistency term and the consistency map.
	update_S_mot(k, n, track);

	// Remove sensor points related to the removed poses.
	for (int j = k; j < n; j++)
		erase_node(true, i, j, tracks);
}


void Posterior::pop_front(int i, int k, const Track::S &tracks)
{
	const Track &track = tracks[i];
	Track::ID id = track.id;

	// Update the length map and the length term.
	lengths[id] -= k;
	S_len -= k;

	// Update the consistency term and the consistency map.
	update_S_mot(0, k, track);

	// Remove sensor points related to the removed poses.
	for (int j = 0; j < k; j++)
		erase_node(true, i, j, tracks);
}


void Posterior::split(const Track &track_1, const Track &track_2)
{
	update_S_len(track_1);
	update_S_len(track_2);
	update_S_mot(track_1);
	update_S_mot(track_2);
}


void Posterior::merge(const Track &track_1, const Track &track_2)
{
	Track::ID id = track_2.id;

	// Update the length term and the length map.
	S_len -= lengths[id];
	lengths.erase(id);

	// Update the consistency term and the consistency map.
	S_mot -= covariances[id];
	covariances.erase(id);

	update_S_len(track_1);
	update_S_mot(track_1);
}


void Posterior::diffuse(int i, int j, const Track::S &tracks)
{
	update_S_mot(i, tracks);
	update_S_ms1(i, j, tracks);
	update_S_ms3(i, j, tracks);
}


void Posterior::shorten(int i, int k, const Track::S &tracks)
{
	const Track &track = tracks[i];
	Track::ID id = track.id;

	// Update the length map and the length term.
	lengths[id] -= k;
	S_len -= k;

	// Update the consistency term and the consistency map.
	update_S_mot(k, track.size(), track);

	// Remove sensor points related to the removed poses.
	for (int j = 0; j < k; j++)
		erase_node(false, i, j, tracks);
}


void Posterior::swap(int i, int j, const Track::S &tracks)
{
	update_S_len(i, tracks);
	update_S_len(j, tracks);

	update_S_mot(i, tracks);
	update_S_mot(j, tracks);
}


void Posterior::update(const Readings &readings)
{
	Zs.erase(readings.front().timestamp);
	Zs.update(readings.back());
}


} // namespace virtual_scan
