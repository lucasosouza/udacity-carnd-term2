/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"
#include "helper_functions.h"

using namespace std;

// initialize random engine
default_random_engine gen;


void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: 

	//Set the number of particles and initialize max_weight 
	num_particles = 200;
	particles.reserve(num_particles);
	max_weight = 1;

	// create a Gaussian distribution for x
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	// Initialize all particles to first position (based on estimates of x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	 
	for (int i=0; i< num_particles; i++) {
		Particle p;
		p.x = dist_x(gen);
		p.y = dist_y(gen);
		p.theta = dist_theta(gen);
		p.weight = 1;
		particles.push_back (p);
	}

	//output: particles vector initialized
	is_initialized = true;

	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

	// TODO: 
	// Add measurements to each particle and add random Gaussian noise.
	for (int i=0; i< num_particles; i++) {
		Particle p = particles[i]; //in array, always by pointer
		
		// update x,y and theta
		double eq1 = velocity/yaw_rate;
		double new_theta = p.theta + yaw_rate * delta_t;
		//issues with numerical stability
		//double new_x = p.x + eq1 * (sin(new_theta) - sin(p.theta));
		//double new_y = p.y + eq1 * (cos(p.theta) - cos(new_theta));
		double new_x = p.x + velocity * cos(p.theta)*delta_t;
		double new_y = p.y + velocity * sin(p.theta)*delta_t;
		// cout << "Old: " << p.x << p.y << p.theta << endl;

		// create normal distributions to add random Gaussian noise
		normal_distribution<double> dist_new_x(new_x, std_pos[0]);
		normal_distribution<double> dist_new_y(new_y, std_pos[1]);
		normal_distribution<double> dist_new_theta(new_theta, std_pos[2]);

		// assign new values to particle, sampling from normal dist
		p.x = dist_new_x(gen);
		p.y = dist_new_y(gen);
		p.theta = dist_new_theta(gen);
		// cout << "New: " << p.x << p.y << p.theta << endl;

		// substitute p in vector
		particles[i] = p;
	}

	//output: particle state (x,y,theta) updated, ready to update weights

	// NOTE: When adding noise you may find normal_distribution and default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the observed measurement to this particular landmark.

	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		vector<LandmarkObs> observations, Map map_landmarks) {

	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. 
	// what is sensor_range for?

	double eq1 = 1/(2*M_PI * std_landmark[0] * std_landmark[1]);
	double eq2 = 2 * std_landmark[0] * std_landmark[0];
	double eq3 = 2 * std_landmark[1] * std_landmark[1];

	//reinit max_weight
	max_weight = 0;

	// iterate through all particles
	for (int i=0; i<particles.size(); i++){

		Particle p = particles[i];
		
		//reinitialize particle weight
		p.weight = 1;

		// new revision
		int n_observations = observations.size();

		vector<int> associations;
		associations.reserve(n_observations);

		vector<double> sense_x;
		sense_x.reserve(n_observations);

		vector<double> sense_y;
		sense_y.reserve(n_observations);

		// transform observations from car coordinate system to map coordinate system
		for (int i=0; i< observations.size(); i++) {
			
			LandmarkObs obs = observations[i];

			// calculate delta
			double cos_theta = cos(p.theta);
			double sin_theta = sin(p.theta);
			double delta_x = cos_theta * obs.x - sin_theta * obs.y;
			double delta_y = sin_theta * obs.x + cos_theta * obs.y;

			cout << "px: " << obs.x << " py: " << obs.y << endl;

			// transform observation
			obs.x = p.x + delta_x;
			obs.y = p.y + delta_y;

			cout << "cx: " << obs.x << " cy: " << obs.y << endl;

			// associate with a landmark 
			Map::single_landmark_s slm;
			double min_distance = 99999; // placeholder for +inf

			// iterate through landmakrs
			for (int j=0; j<= map_landmarks.landmark_list.size(); j++) {

				Map::single_landmark_s lm = map_landmarks.landmark_list[j];

				// for each landmark, calculate eclidean distance from obs and landmark
				double distance = dist(obs.x, obs.y, lm.x_f, lm.y_f);
				if (distance < min_distance) {
					min_distance = distance;
					slm = lm;
					// cout << slm.id_i << endl;
					// cout << slm.x_f << endl;
					// cout << slm.y_f << endl;
				}
			}

			// cout << "min_distance: " << dist(obs.x, obs.y, slm.x_f, slm.y_f) << endl;
			// cout << slm.id_i << endl;
			// cout << slm.x_f << endl;
			// cout << slm.y_f << endl;

			// int j = 0;
			// for (i=0; i<1e7; i++) {
			// 	j +=i; 
			// }

			// calculate measurement Multivariate gaussian probability
			double eq4 = pow(obs.x - slm.x_f, 2)/eq2 + pow(obs.y - slm.y_f, 2)/eq3; 
			double prob = eq1 * exp(-eq4);
			//std::cout << "Min distance: " << dist(obs.x,obs.y,slm.x_f,slm.y_f) << std::endl;
			//cout << "Prob: " << prob << endl;

			// update weight
			p.weight *= prob;

			// new version: save association info for simulator
			associations.push_back(slm.id_i);
			sense_x.push_back(slm.x_f);
			sense_y.push_back(slm.y_f);

		}

		// update max weight
		if (p.weight > max_weight) {
			max_weight = p.weight;
		}

		//update associations info
		p = SetAssociations(p, associations, sense_x, sense_y);

		// substitute p in vector
		particles[i] = p;

	}

	//output: particle weights updated, ready for resample

	// You can read more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
}

void ParticleFilter::resample() {

	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// use Sebastian's algorithm of the wheel

    vector<Particle> new_particles;
    int index = floor(rand() % num_particles);
    double beta = 0;
    for (int i=0; i<num_particles; i++) {
    	beta += fmod(rand(), (2*max_weight));
    	while (beta > particles[index].weight) {
    		beta -= particles[index].weight;
    		index = (index + 1) % num_particles;
    	}
    	new_particles.push_back (particles[index]);
    }

    // output: update particles vector
    particles = new_particles;
    //cout << "Particles after resample: " << particles.size() << endl;

	// NOTE: You may find discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

// new code appeared after - no idea what changed

Particle ParticleFilter::SetAssociations(Particle particle, vector<int> associations, vector<double> sense_x, vector<double> sense_y)
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
