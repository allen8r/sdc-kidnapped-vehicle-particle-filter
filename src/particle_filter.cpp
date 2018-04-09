/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 *   Completed: March 4, 2018
 *          By: Allen Lau
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
	// Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	
	// Set number of particles
	num_particles = 100;

	// Initialize size of weights vector
	weights.resize(num_particles);

	// Consider checking for null array and array size to prevent runtime array index errors
	double std_x = std[0];
	double std_y = std[1];
	double std_theta = std[2];
	
	// Normal Gaussian distributions for x, y, and theta
	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(normalizeAngle(theta), std_theta);

	// Create particles using the Gaussians for sampling
	default_random_engine rand_gen;
	
	for (int i = 0; i < num_particles; i++) {
		double sample_x = dist_x(rand_gen);
		double sample_y = dist_y(rand_gen);
		double sample_theta = dist_theta(rand_gen);

		Particle p = Particle();
		p.id = i;
		p.x = sample_x;
		p.y = sample_y;
		p.theta = normalizeAngle(sample_theta);
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
	// Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	default_random_engine rand_gen;

	double std_x = std_pos[0];
	double std_y = std_pos[1];
	double std_theta = std_pos[2];

	// Normal Gaussian distributions for x, y, and theta
	normal_distribution<double> noisy_x(0.0, std_x);
	normal_distribution<double> noisy_y(0.0, std_y);
	normal_distribution<double> noisy_theta(0.0, std_theta);
	
	double v_dt = velocity * delta_t;
	
	for (int i = 0; i < num_particles; i++) {
		double theta = particles[i].theta;

		// Motion model
		if (fabs(yaw_rate) < 0.0001) { // yaw rate is zero; constant yaw
			particles[i].x = particles[i].x + v_dt * cos(theta);
			particles[i].y = particles[i].y + v_dt * sin(theta);

		} else {	// non-zero yaw rate
			double v_over_yaw_rate = velocity / yaw_rate;
			double yaw_delta = yaw_rate * delta_t;	
			double theta_yaw_delta = theta + yaw_rate * delta_t;
			particles[i].x = particles[i].x + v_over_yaw_rate * (sin(theta_yaw_delta) - sin(theta));
			particles[i].y = particles[i].y + v_over_yaw_rate * (cos(theta) - cos(theta_yaw_delta));
			particles[i].theta = theta_yaw_delta;
		}
		// Add Gaussian noise
		particles[i].x += noisy_x(rand_gen);
		particles[i].y += noisy_y(rand_gen);
		particles[i].theta += noisy_theta(rand_gen);
	}

}

/**
 * NOT USED
 */
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> potentials, std::vector<LandmarkObs>& observations) {
	// TODO: Find the potential measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// Update the weights of each particle using a mult-variate Gaussian distribution. You can read
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
		
		vector<LandmarkObs> transformed;
		for (int i = 0; i < observations.size(); i++) {
			// Transform observation from Vehicle's coordinates to Map's coordinates with respect to particle
			transformed.push_back(transformToMapCoords(observations[i], particles[p]));
		}

		// Check which landmarks are within sensor range
		vector<LandmarkObs> potentials;
		for (int m = 0; m < map_landmarks.landmark_list.size(); m++) {
			if (dist(particles[p].x, particles[p].y,
							map_landmarks.landmark_list[m].x_f,
							map_landmarks.landmark_list[m].y_f) <= sensor_range) {

				Map::single_landmark_s lm =  map_landmarks.landmark_list[m];
					LandmarkObs landmark;
					landmark.id = lm.id_i;
					landmark.x = lm.x_f;
					landmark.y = lm.y_f;
					potentials.push_back(landmark);
			}
		}

		// Find nearest landmarks (from potentials) to transformed observations
		//  and create particle associations to closest landmarks
		if (potentials.size() > 0) {
			vector<int> associations;
			vector<double> obs_x;
			vector<double> obs_y;
			for (int i = 0; i < transformed.size(); i++) {
				double minDist = std::numeric_limits<double>::max();
				int bestIndex = 0;
				for (int p = 0; p < potentials.size(); p++) {
					double d = dist(potentials[p].x, potentials[p].y, transformed[i].x, transformed[i].y);
					if (d < minDist) {
						minDist = d;
						bestIndex = p;
					}
				}
				associations.push_back(potentials[bestIndex].id);
				obs_x.push_back(transformed[i].x);
				obs_y.push_back(transformed[i].y);
			}

			particles[p].associations = associations;
			particles[p].sense_x = obs_x;
			particles[p].sense_y = obs_y;
		
			// Calculate particle's new weight
			double particle_weight = 1.0; // product of particles probability densities
			for (int i = 0; i < particles[p].associations.size(); i++) {
				int landmark_id = particles[p].associations[i];
				// NOTE: landmark ids are one-based while list of landmarks are zero-based indexed,
				//  thus need to use landmark_id-1 to retrieve the correct associated landmark
				Map::single_landmark_s landmark = map_landmarks.landmark_list[landmark_id-1];
				double x_lm = landmark.x_f;
				double y_lm = landmark.y_f;
				double x_obs = particles[p].sense_x[i];
				double y_obs = particles[p].sense_y[i];

				double prob = multiGaussPd(x_obs, y_obs, x_lm, y_lm, std_landmark[0], std_landmark[1]);

				particle_weight *= prob;
			}

			particles[p].weight = particle_weight;
			weights[p] = particle_weight;
		}
	}

}

void ParticleFilter::resample() {
	// Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	// Using "resampling wheel" algorithm from Sebastian's lecture
	vector<Particle> resampled(num_particles);

	default_random_engine rand_gen;
	uniform_real_distribution<double> urdis(0, 1.0);
	int index = int(urdis(rand_gen) * num_particles);
	double weight_max = *max_element(weights.begin(),weights.end());
	double beta = 0.0;

	for (int i = 0; i < num_particles; i++) {
			beta += urdis(rand_gen) * 2.0 * weight_max;
			while (weights[index] < beta) {
					beta -= weights[index];
					index = (index + 1) % num_particles;
			}
			resampled[i] = particles[index];
	}
			
	particles = resampled;
	
}

/**
 * NOT USED
 */
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

/**
 * USED in main.cpp
 */
string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

/**
 * USED in main.cpp
 */
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

/**
 * USED in main.cpp
 */
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}


LandmarkObs ParticleFilter::transformToMapCoords(LandmarkObs obs, Particle p) {
	LandmarkObs mapCoords;
	double theta = normalizeAngle(p.theta);
	mapCoords.x = p.x + (obs.x * cos(theta)) - (obs.y * sin(theta));
	mapCoords.y = p.y + (obs.x * sin(theta)) + (obs.y * cos(theta));
	return mapCoords;
}

/**
 * Multivariate Gaussian Probability Density function
 */
double ParticleFilter::multiGaussPd(double x, double y, double mux, double muy,
																		double stdx, double stdy) {
	return (1/(2*M_PI*stdx*stdy))
					* exp(-((pow(x-mux,2)/pow(2*stdx,2)) + (pow(y-muy,2)/pow(2*stdy,2))));
}
