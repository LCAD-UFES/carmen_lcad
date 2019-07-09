#include "message_interpolation.h"

template <typename M1, typename M2>
class MessageInterpolation {

public:

	MessageInterpolation(int list_size);
	void AddMessageToInterpolationList(M1 *message);
	M1 InterpolateMessages(M2 *message);
	int GetListSize() { return interpolation_list_size; }

	virtual ~MessageInterpolation();

private:
	int interpolation_list_counter;
	int interpolation_list_size;
	M1* interpolation_list;
};

template <typename M1, typename M2>
MessageInterpolation<M1, M2>::MessageInterpolation(int list_size) {
	this->interpolation_list = (M1* ) calloc (list_size, sizeof(M1));
	carmen_test_alloc(this->interpolation_list);

	this->interpolation_list_size = list_size;
	this->interpolation_list_counter = 0;
}

template <typename M1, typename M2>
void MessageInterpolation<M1, M2>::AddMessageToInterpolationList(M1 *message)
{
	this->interpolation_list[this->interpolation_list_counter] = *message;

	if(this->interpolation_list_counter == this->interpolation_list_size - 1)
	{
		this->interpolation_list_counter = 0;
		return;
	}

	this->interpolation_list_counter++;
}

template <typename M1, typename M2>
M1 MessageInterpolation<M1, M2>::InterpolateMessages(M2 *message)
{
	M1 nearest_message;
	int nearest_index = 0;
	double curr_timestamp_difference;
	double min_timestamp_difference = 99999999.0;

	for (int i = 0; i < this->interpolation_list_size; i++)
	{
		curr_timestamp_difference = fabs(this->interpolation_list[i].timestamp - message->timestamp);

		if(curr_timestamp_difference < min_timestamp_difference)
		{
			min_timestamp_difference = curr_timestamp_difference;
			nearest_index = i;
		}
	}

	nearest_message = this->interpolation_list[nearest_index];
	return nearest_message;
}

template <typename M1, typename M2>
MessageInterpolation<M1, M2>::~MessageInterpolation() {

}
