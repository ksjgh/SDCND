/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 *
 * modified on: Jul 21, 2017
 *      Author: Sung-jin Kwon
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

static int MAX_PARTICLES = 80;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	// Set number of initial particles
	num_particles = MAX_PARTICLES;

	// Resize particles vector
	particles.resize(num_particles);

	// for random particle generation based on initial measurement
	std::default_random_engine gen;

	std::normal_distribution<double> N_x(x,std[0]);
	std::normal_distribution<double> N_y(y,std[1]);
	std::normal_distribution<double> N_theta(theta,std[2]);

	// generate particles around initial measurement point
	for(auto &p : particles){
		// p.id = 0;
		p.x = N_x(gen);
		p.y = N_y(gen);
		p.theta = N_theta(gen);
		p.weight = 1.0;
	}

	is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/


	// assume motion process noise as gaussian
	default_random_engine gen;
	normal_distribution<double> N_x(0, std_pos[0]);
  normal_distribution<double> N_y(0, std_pos[1]);
  normal_distribution<double> N_theta(0, std_pos[2]);

	// update new postion of each particle based on CTRV motion model
	for(auto &p : particles)
	{
		// calculate new position
    if (fabs(yaw_rate) < 0.00001) {  // to avoid divide by zero by yaw_rate
      p.x += velocity * delta_t * cos(p.theta);
      p.y += velocity * delta_t * sin(p.theta);
			// p.theta += 0; // no changes in theta
    }
    else {
			p.x += velocity / yaw_rate * ( sin( p.theta + yaw_rate*delta_t ) - sin(p.theta) );
			p.y += velocity / yaw_rate * ( cos( p.theta ) - cos( p.theta + yaw_rate*delta_t ) );
			p.theta += yaw_rate * delta_t;
    }

    // add motion noise(control noise)
		p.x += N_x(gen);
		p.y += N_y(gen);
		p.theta += N_theta(gen);
	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.


	// for each measurement point in observations, find nearest landmark in 'predicted' and gets its landmark id
	// 'predicted' is array of landmarks near the particle
	for(auto &obs: observations){ // for each measurement data
    double min_dist = std::numeric_limits<float>::max();

    for(const auto &lm: predicted){  // for landmark in filtered landmarks within sensor range
      double distance = dist(obs.x, obs.y, lm.x, lm.y);
      if( min_dist > distance){
        min_dist = distance;
        obs.id = lm.id;
			}
		}
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		std::vector<LandmarkObs> observations, Map map_landmarks) {
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

	// calculate weight for particle
	weights.clear();
	for(auto &p : particles) {

		// // filtering landmarks within circle of sensor range
		// vector<LandmarkObs> filtered_landmarks;
    // for(const auto& lm: map_landmarks.landmark_list){
    //   double distance = dist(p.x, p.y, lm.x_f, lm.y_f);
    //   if( distance < sensor_range){
    //     filtered_landmarks.push_back(LandmarkObs{lm.id_i, lm.x_f, lm.y_f});
    //   }
    // }

		// filtering landmarks within sensor square range , for faster calculation
		vector<LandmarkObs> filtered_landmarks;

		// select landmarks within square region of particle
		for(const auto &lm: map_landmarks.landmark_list){
      if( fabs(lm.x_f-p.x) < sensor_range && fabs(lm.y_f-p.y) < sensor_range){
        filtered_landmarks.push_back(LandmarkObs{lm.id_i, lm.x_f, lm.y_f});
      }
    }

		// convert measurement data coordinate to map coordiante
		vector<LandmarkObs> converted_observations;
    for(const auto &obs: observations){
      LandmarkObs tmp;
      tmp.x = obs.x * cos(p.theta) - obs.y * sin(p.theta) + p.x;
      tmp.y = obs.x * sin(p.theta) + obs.y * cos(p.theta) + p.y;
			// tmp.id , will be set in dataAssociation()
			converted_observations.push_back(tmp);
		}

		// find nearest landmark  to each observation
		dataAssociation(filtered_landmarks, converted_observations);

		// calculate weight using multi-variate gaussian distribution
		// distance between observation data and associated landmark
		//
		p.weight = 1.0;
		for(const auto& c_obs: converted_observations){

			// get associated landmark
    	Map::single_landmark_s lm = map_landmarks.landmark_list.at(c_obs.id-1);

			// calculate weight using multi-variate gaussian distribution
      double x_diff = pow(c_obs.x - lm.x_f, 2) / (2 * pow(std_landmark[0], 2));
      double y_diff = pow(c_obs.y - lm.y_f, 2) / (2 * pow(std_landmark[1], 2));
      double weight_i = exp(-(x_diff + y_diff)) / (2 * M_PI * std_landmark[0] * std_landmark[1]);
      p.weight *=  weight_i;
    }

    weights.push_back(p.weight);
	}
}


void ParticleFilter::resample() {
  // DONE: Resample particles with replacement with probability proportional to their weight.
  // NOTE: You may find std::discrete_distribution helpful here.
  //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	default_random_engine gen;

  discrete_distribution<int> distribution(weights.begin(), weights.end());

  vector<Particle> resample_particles;

  for (int i = 0; i < num_particles; i++) {
      resample_particles.push_back(particles[distribution(gen)]);
  }
  particles = resample_particles;

}



Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
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
