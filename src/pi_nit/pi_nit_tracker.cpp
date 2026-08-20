#include "pi_nit_tracker.hpp"

#include <algorithm>


PiNitTracker::PiNitTracker(double iou_threshold, int max_age_frames) :
		iou_threshold_(iou_threshold), max_age_frames_(max_age_frames), next_id_(1)
{
}


void
PiNitTracker::reset()
{
	tracks_.clear();
	next_id_ = 1;
}


double
PiNitTracker::iou(const Track &track, const pi_nit_detection_t &detection)
{
	double inter_x1 = std::max((double) track.x1, (double) detection.x1);
	double inter_y1 = std::max((double) track.y1, (double) detection.y1);
	double inter_x2 = std::min((double) track.x2, (double) detection.x2);
	double inter_y2 = std::min((double) track.y2, (double) detection.y2);

	double inter_w = inter_x2 - inter_x1;
	double inter_h = inter_y2 - inter_y1;
	if ((inter_w <= 0.0) || (inter_h <= 0.0))
		return (0.0);

	double intersection = inter_w * inter_h;
	double area_track = (double) (track.x2 - track.x1) * (double) (track.y2 - track.y1);
	double area_detection = (double) (detection.x2 - detection.x1) * (double) (detection.y2 - detection.y1);
	double union_area = area_track + area_detection - intersection;

	if (union_area <= 0.0)
		return (0.0);

	return (intersection / union_area);
}


std::vector<int>
PiNitTracker::update(const std::vector<pi_nit_detection_t> &detections)
{
	std::vector<int> track_ids(detections.size(), 0);
	std::vector<bool> track_used(tracks_.size(), false);
	std::vector<bool> detection_used(detections.size(), false);

	// Associacao gulosa: em cada rodada casa o par (track, deteccao) de maior
	// IoU ainda livre, ate nao haver par acima do limiar.
	while (true)
	{
		double best_iou = iou_threshold_;
		size_t best_track = 0;
		size_t best_detection = 0;
		bool found = false;

		for (size_t t = 0; t < tracks_.size(); t++)
		{
			if (track_used[t])
				continue;

			for (size_t d = 0; d < detections.size(); d++)
			{
				if (detection_used[d] || (tracks_[t].class_id != detections[d].class_id))
					continue;

				double value = iou(tracks_[t], detections[d]);
				if (value > best_iou)
				{
					best_iou = value;
					best_track = t;
					best_detection = d;
					found = true;
				}
			}
		}

		if (!found)
			break;

		track_used[best_track] = true;
		detection_used[best_detection] = true;

		tracks_[best_track].x1 = detections[best_detection].x1;
		tracks_[best_track].y1 = detections[best_detection].y1;
		tracks_[best_track].x2 = detections[best_detection].x2;
		tracks_[best_track].y2 = detections[best_detection].y2;
		tracks_[best_track].age = 0;

		track_ids[best_detection] = tracks_[best_track].id;
	}

	// Deteccoes sem par viram tracks novos
	for (size_t d = 0; d < detections.size(); d++)
	{
		if (detection_used[d])
			continue;

		Track track;
		track.id = next_id_++;
		track.class_id = detections[d].class_id;
		track.x1 = detections[d].x1;
		track.y1 = detections[d].y1;
		track.x2 = detections[d].x2;
		track.y2 = detections[d].y2;
		track.age = 0;

		if (next_id_ <= 0)   // overflow: volta a contar (0 e' reservado)
			next_id_ = 1;

		tracks_.push_back(track);
		track_ids[d] = track.id;
	}

	// Envelhece e remove tracks nao vistos ha max_age_frames_ frames
	std::vector<Track> surviving;
	surviving.reserve(tracks_.size());
	for (size_t t = 0; t < tracks_.size(); t++)
	{
		if (t < track_used.size() && !track_used[t])
			tracks_[t].age++;

		if (tracks_[t].age <= max_age_frames_)
			surviving.push_back(tracks_[t]);
	}
	tracks_.swap(surviving);

	return (track_ids);
}
