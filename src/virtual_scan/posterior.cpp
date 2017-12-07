#include "posterior.h"

#include "parameters.h"

namespace virtual_scan
{


Posterior::Posterior():
	lengths(1, 0.0),
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


void Posterior::update_S_len(size_t n, const Track::S &tracks)
{
	auto length = lengths.begin();
	for (size_t i = 0; i < n; i++)
	{
		if (lengths.size() <= i)
			lengths.push_back(0.0);

		length++;
	}

	S_len -= *length;
	*length = tracks.size();
	S_len += *length;
}


void Posterior::update_S_mot(int i, const Track::S &tracks)
{
}


void Posterior::update_S_ms1(int i, int j, const Track::S &tracks)
{
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
