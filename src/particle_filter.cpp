/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang, Allen Lau
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	num_particles = 20;

	// consider checking for null array and array size to prevent runtime array index errors
	double std_x = std[0];
	double std_y = std[1];
	double std_theta = std[2];
	
	default_random_engine rand_gen;

	// Normal Gaussian distributions for x, y, and theta
	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);

	// Create particles using the Gaussians for sampling - introducing noise
	for (int i = 0; i < num_particles; i++) {
		double sample_x = dist_x(rand_gen);
		double sample_y = dist_y(rand_gen);
		double sample_theta = dist_theta(rand_gen);

		Particle p = Particle();
		p.id = i;
		p.x = sample_x;
		p.y = sample_y;
		p.theta = sample_theta;
		p.weight = 1;

		particles.push_back(p);
	}

	if (num_particles != particles.size()) {
		std::cout << "Error: Number of particles does not match num_particles! Check ParticleFilter::init()" 
			<< std::endl;
	}

	is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	if (fabs(yaw_rate) < 0.0001) {
		std::cout << "Error: division by zero, bad yaw_rate provided to ParticleFilter::prediction()!"
			<< std::endl;
		return;
	}

	double v_over_yaw_rate = velocity / yaw_rate;

	double std_x = std_pos[0];
	double std_y = std_pos[1];
	double std_theta = std_pos[2];

	default_random_engine rand_gen;
	
	for (int i = 0; i < num_particles; i++) {
		Particle p = particles[i];
		p.x = p.x + v_over_yaw_rate * (sin(p.theta + yaw_rate * delta_t) - sin(p.theta));
		p.y = p.y + v_over_yaw_rate * (cos(p.theta) - cos(p.theta + yaw_rate * delta_t));
		p.theta = p.theta + yaw_rate * delta_t;

		// Add Gaussian noise
		// Normal Gaussian distributions for x, y, and theta
		normal_distribution<double> dist_x(p.x, std_x);
		normal_distribution<double> dist_y(p.y, std_y);
		normal_distribution<double> dist_theta(p.theta, std_theta);
		
		p.x = dist_x(rand_gen);
		p.y = dist_y(rand_gen);
		p.theta = dist_theta(rand_gen);

	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> potentials, std::vector<LandmarkObs>& observations) {
	// TODO: Find the potential measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	for (int i = 0; i < observations.size(); i++) {
		double minDist = std::numeric_limits<double>::max();
		int bestIndex = 0;
		for (int p = 0; p < potentials.size(); p++) {
			double d = dist(potentials[p].x, potentials[p].y, observations[i].x, observations[i].y);
			if (d < minDist) {
				minDist = d;
				bestIndex = p;
			}
		}
		observations[i].id = potentials[bestIndex].id;
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	
	for (int p = 0; p < particles.size(); p++) {
		Particle pt = particles[p];
		vector<LandmarkObs> potentials;
		vector<LandmarkObs> transformed;
		for (int i = 0; i < observations.size(); i++) {
			// Transform observation from Vehicle's coordinates to Map's coordinates with respect to particle
			LandmarkObs transformedObs = transformToMapCoords(observations[i], pt);
			transformed.push_back(transformedObs);

			// Check which landmarks are within sensor range
			for (int m = 0; m < map_landmarks.landmark_list.size(); m++) {
				if (dist(transformedObs.x, transformedObs.y, pt.x, pt.y) <= sensor_range) {
					LandmarkObs landmark;
					landmark.id = landmark.x = map_landmarks.landmark_list[m].id_i;
					landmark.x = map_landmarks.landmark_list[m].x_f;
					landmark.y = map_landmarks.landmark_list[m].y_f;
					potentials.push_back(landmark);
				}
			}
		}

		// find nearest landmarks (from potentials) to transformed observations
		dataAssociation(potentials, transformed);
		
		// save associations to particle
		vector<int> associations;
		vector<double> obs_x;
		vector<double> obs_y;
		for (int t = 0; t < transformed.size(); t++) {
			associations.push_back(transformed[t].id);
			obs_x.push_back(transformed[t].x);
			obs_y.push_back(transformed[t].y);
		}
		SetAssociations(pt, associations, obs_x, obs_y);

		// Calculate particle's new weight
		double particle_weight = 1; // product of particles probability densities
		for (int i = 0; i < pt.associations.size(); i++) {
			int landmark_id = pt.associations[i];
			Map::single_landmark_s landmark = map_landmarks.landmark_list[landmark_id-1];
			double x_lm = landmark.x_f;
			double y_lm = landmark.y_f;
			double x_obs = pt.sense_x[i];
			double y_obs = pt.sense_y[i];

			particle_weight *= multiGaussPd(x_obs, y_obs, x_lm, y_lm, std_landmark[0], std_landmark[1]);
		}

		pt.weight = particle_weight;

	}

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

LandmarkObs ParticleFilter::transformToMapCoords(LandmarkObs observation, Particle p) {
	LandmarkObs mapCoords;
	mapCoords.x = p.x + (cos(p.theta) * observation.x) - (sin(p.theta * observation.y));
	mapCoords.y = p.y + (sin(p.theta) * observation.x) + (cos(p.theta * observation.y));
	return mapCoords;
}

double ParticleFilter::multiGaussPd(double x, double y, double mux, double muy,
																		double stdx, double stdy) {
	return (1/(2*M_PI*stdx*stdy))
					* exp(-((pow(x-mux,2)/pow(2*stdx,2)) + (pow(y-muy,2)/pow(2*stdy,2))));
}
