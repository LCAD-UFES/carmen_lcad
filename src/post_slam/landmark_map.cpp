/*
 * landmark_map.cpp
 *
 *  Created on: Apr 12, 2013
 *      Author: filipe
 */
#include "landmark.h"
#include "landmark_map.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

namespace post_slam
{
	double LandmarkMapParams::max_post_radius = 5.0;

	LandmarkMap::LandmarkMap()
	{
	}


	LandmarkMap::~LandmarkMap()
	{
	}


	void
	LandmarkMap::add(vector<Landmark*> *new_landmarks)
	{
		unsigned int i;

		for (i = 0; i < new_landmarks->size(); i++)
			this->add(new_landmarks->at(i));
	}


	void
	LandmarkMap::add(Landmark *landmark)
	{
		unsigned int i;

		for(i = 0; i < landmarks.size(); i++)
		{
			// se a feature estiver a uma distancia menor que um limiar de alguma outra
			// feature ja adicionada, considero que eh outra visualizacao da mesma feature
			if (sqrt(pow(landmark->x - landmarks[i]->x, 2) + pow(landmark->y - landmarks[i]->y, 2)) < LandmarkMapParams::max_post_radius)
			{
				// TODO: checar se isso eh importante:
				// nesse caso, atualizo a pose e o raio da feature,
				// adicionando a informacao da nova visualizacao
				//features[i].x = (features[i].x + pose_x) / 2.0;
				//features[i].y = (features[i].y + pose_y) / 2.0;
				//features[i].radius = 1.0; //(features[i].radius + radius) / 2.0;

				return;
			}
		}

		landmarks.push_back(new Landmark(*landmark));
	}


	Landmark*
	LandmarkMap::find_nearest_landmark(Landmark* landmark)
	{
		unsigned int i;

		double distance;
		int first_iteraction = 1;
		int closest_landmark_index = 0;
		double closest_landmark_distance = 0;

		for (i = 0; i < landmarks.size(); i++)
		{
			distance = sqrt(pow(landmarks[i]->x - landmark->x, 2) + pow(landmarks[i]->y - landmark->y, 2));

			if (first_iteraction)
			{
				closest_landmark_distance = distance;
				closest_landmark_index = i;
				first_iteraction = 0;
			}

			if (distance < closest_landmark_distance)
			{
				closest_landmark_distance = distance;
				closest_landmark_index = i;
			}
		}

		return landmarks[closest_landmark_index];
	}


	Landmark*
	LandmarkMap::match(Landmark* landmark)
	{
		return find_nearest_landmark(landmark);
	}


	vector<Landmark*>
	LandmarkMap::match(vector<Landmark*> *landmarks_to_match)
	{
		unsigned int i;
		vector<Landmark*> matches;

		printf("03\n");

		for (i = 0; i < landmarks_to_match->size(); i++)
			matches.push_back(find_nearest_landmark(landmarks_to_match->at(i)));

		return matches;
	}


	void
	LandmarkMap::save(char *filename)
	{
		FILE *f;
		unsigned int i;

		f = fopen(filename, "w");

		if (f == NULL)
			exit(printf("Error: Unable to save the landmark map to the file '%s'\n", filename));

		// TODO: salvar alguma informacao da feature para fazer matching
		for (i = 0; i < landmarks.size(); i++)
			fprintf(f, "%lf %lf\n", landmarks[i]->x, landmarks[i]->y);

		fclose(f);
	}


	void
	LandmarkMap::load(char *filename)
	{
		FILE *f;
		double x;
		double y;

		f = fopen(filename, "r");

		if (f == NULL)
			exit(printf("Error: Unable to load the landmark map from the file '%s'\n", filename));

		while (!feof(f))
		{
			fscanf(f, "\n%lf %lf\n", &x, &y);
			landmarks.push_back(new Landmark(x, y, 1.0));
		}

		fclose(f);

		if (landmarks.size() == 0)
			exit(printf("Error: Empty map\n"));
		else
			printf("Num landmarks loaded: %ld\n", landmarks.size());
	}
}
