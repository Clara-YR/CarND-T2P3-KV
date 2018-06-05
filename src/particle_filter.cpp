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

    // initialize a random engine
    default_random_engine gen;

    // set the number of particles
    num_particles = 100;

    // create normal distributions for x, y and theta
    // use it to add random Gaussian noise later
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);

    // generate particles with normal distribution with mean on GPS values
    for(int i=0; i<num_particles; ++i) {

        Particle particle_i;

        particle_i.x = dist_x(gen);
        particle_i.y = dist_y(gen);
        particle_i.theta = dist_theta(gen);
        particle_i.weight = 1.0;

        particles.push_back(particle_i);
    }

    is_initialized = true;
    cout << " Initialization Done " << endl;
    return ;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

    // initialize a random engine
    default_random_engine gen;

    for (int i=0; i<num_particles; ++i) {
        Particle *p = &particles[i]; // get address of particle to update

        //predictions of x, y, theta
        double x_p, y_p, theta_p;

        // avoid division by 0
        if (fabs(yaw_rate) > 0.001) {
            x_p = p->x + (velocity/yaw_rate) * (sin(p->theta + yaw_rate*delta_t) - sin(p->theta));
            y_p = p->y + (velocity/yaw_rate) * (cos(p->theta) - cos(p->theta + yaw_rate*delta_t));
            theta_p = p->theta + (yaw_rate * delta_t);

        } else{
            x_p = p->x + velocity * delta_t * cos(p->theta);
            y_p = p->y + velocity * delta_t * sin(p->theta);
            theta_p = p->theta + (yaw_rate * delta_t);
        }

        // create Gaussian normal distributions for px_p, py_p and yaw_p
        normal_distribution<double> dist_x_p(x_p, std_pos[0]);
        normal_distribution<double> dist_y_p(y_p, std_pos[1]);
        normal_distribution<double> dist_theta_p(theta_p, std_pos[2]);

        // add noise to the predicted measurement for the i-th particle
        p->x = dist_x_p(gen);
        p->y = dist_y_p(gen);
        p->theta = dist_theta_p(gen);
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

    for (int n=0; n<observations.size(); ++n) {
        LandmarkObs *ob = &observations[n];
        // initialize the closest index and distance
        int index_min;
        double dist_min = numeric_limits<double>::max();

        // find the closest predicted measurement for the n-th observation
        for (int i=0; i<predicted.size(); ++n){
            double distance = dist(predicted[i].x, predicted[i].y, ob->x, ob->y);
            if (distance < dist_min){
                dist_min = distance;
                index_min = i;
            }
        }
        // assign the observation measurement to the i_min-th predicted measurement
        predicted[index_min] = observations[n];
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

    double sum_w_update = 0.0;  // sum of weights to normalize each weight
    for (int i=0; i<num_particles; ++i) {
        Particle *p = &particles[i];  // get the i-th particle address to update
        double w_update = 1.0;  // weights to be updated for the i-th particle at time t

        vector<LandmarkObs> obs_i;
        for (int n = 0; n < observations.size(); ++n) {

            /* TRANSFORM
             * transform the given sensor landmark observations from the VEHICLE'S coordinate system
             * into the MAP'S coodinate system, which the particles are located according to.
             */
            const LandmarkObs *ob_car = &observations[n];  // get the n-th observation address to transform
            LandmarkObs ob_map;  // observations transformed form car's into map's coordinate
            ob_map.x = p->x + (cos(p->theta) * ob_car->x) - (sin(p->theta) * ob_car->y);
            ob_map.y = p->y + (sin(p->theta) * ob_car->x) + (cos(p->theta) * ob_car->y);

            /* ASSOCIATION
             * associating the transformed observations(in MAP'S coordinate system)
             * with the nearest landmark on the map.
             */
            double distance_min = numeric_limits<double>::max();
            int id_min = -1;   // the id for the nearest map landmark
            for (int m = 0; m < map_landmarks.landmark_list.size(); ++m) {
                double distance = dist(ob_map.x, ob_map.y, map_landmarks.landmark_list[m].x_f,
                                       map_landmarks.landmark_list[m].y_f);
                if (distance < distance_min) {
                    distance_min = distance;
                    id_min = m;
                }
            }

            /* UPDATE WEIGHTS
             * using multivariate-Gaussian Probability
             */
            // calculate normalization term
            double guass_norm = 1.0 / (2.0 * M_PI * std_landmark[0] * std_landmark[1]);

            // calculate exponent taking the nearest landmark as the mean
            double mu_x = map_landmarks.landmark_list[id_min].x_f;
            double mu_y = map_landmarks.landmark_list[id_min].y_f;
            double exponent = pow(ob_map.x-mu_x, 2)/(2*pow(std_landmark[0],2)) + pow(ob_map.y-mu_y, 2)/(2*pow(std_landmark[1],2));

            // P(i_th particle state at t|given the n-th observation at t)
            w_update *= guass_norm * exp(-exponent);  // mulitply likelihood given each observation
        }
        p->weight = w_update;
        sum_w_update += w_update;  // add updated weights for each circle
    }
    // normalize all weights
    for(int i=0; i<num_particles; ++i) {
        particles[i].weight /= sum_w_update;
    }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

    // creates weights vector list
    vector<double> weights;
    for (int i=0; i<num_particles; i++) {  // For each particle

        //cout << particles[i].weight << endl;
        weights.push_back(particles[i].weight);
    }

    // initialize a random engine
    default_random_engine gen;
    // create distribution
    discrete_distribution<double> dist(weights.begin(), weights.end());

    // run resampling wheel
    vector<Particle> resample_particles;
    for (int i=0; i<num_particles; ++i) {
        int index = dist(gen);
        resample_particles.push_back(particles[index]);
    }
    particles = resample_particles;
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
