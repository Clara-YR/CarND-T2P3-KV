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
#include <cmath> //
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include <random>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

    cout << "Initial PF ..." << endl;
    cout << "x=" << x <<" ,y=" << y << " ,theta="<< theta << endl;

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

    cout << "Prediction ..." << endl;
    // initialize a random engine
    default_random_engine gen;
    // create a vector list to store particle measurement prediction
    vector<Particle> predictions;
    //vector<Particle> pred_measurements;

    for (int i=0; i<num_particles; ++i) {
        // predict the i-th particle

        predictions[i].id = i;  // predicitons[i] is the prediction of particles[i]
        double yaw, x_p, y_p, yaw_p;
        yaw = particles[i].theta;
        yaw_p = yaw;

        // avoid division by 0
        if (fabs(yaw_rate) > 0.001) {
            x_p= particles[i].x + (velocity/yaw_rate) * (sin(yaw + yaw_rate*delta_t) - sin(yaw));
            y_p = particles[i].y + (velocity/yaw_rate) * (cos(yaw) - cos(yaw + yaw_rate*delta_t));
        } else{
            x_p = particles[i].x + velocity * delta_t * cos(yaw);
            y_p = particles[i].y + velocity * delta_t * sin(yaw);
        }
        //cout << "Dx = " << (particles[i].x - px_p) << ", Dy = " << (particles[i].y - py_p) << endl;

        // create Gaussian normal distributions for px_p, py_p and yaw_p
        normal_distribution<double> pred_x(x_p, std_pos[0]);
        normal_distribution<double> pred_y(y_p, std_pos[1]);
        normal_distribution<double> pred_yaw(yaw_p, std_pos[2]);

        // add noise to the predicted measurement for the i-th particle
        particles[i].x = pred_x(gen);
        particles[i].y = pred_y(gen);
        particles[i].theta = pred_yaw(gen);

        //cout << "particles[i](x, y, theta) = " << particles[i].x << "\t" << particles[i].y << "\t" << particles[i].theta <<endl;
    }
    cout << "Prediction done" << endl;
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

    cout << "Data Association ..." << endl;
    cout << "predicted size = " << predicted.size() << endl;
    cout <<"observations size = " << observations.size() << endl;
    for (int i=0; i<observations.size(); ++i) {

        // find the neareast neighbour(prediction) for each observation_i
        cout << "i = " << i<< endl;
        double dist_nearest = 50;
        double dist_ij;
        int nearest_i;

        for (int j=0; j<predicted.size(); ++j) {
            // compare each predicted measurement with observation
            double dist_x = predicted[j].x - observations[i].x;
            double dist_y = predicted[j].y - observations[i].y;
            dist_ij = sqrt(dist_x*dist_x + dist_y*dist_y);

            if (dist_nearest >= dist_ij) {
                dist_nearest = dist_ij;
                // get the index of the nearest predicted measurement to the i-th observation
                nearest_i = j;
                cout << "dist_nearest = " << dist_nearest << endl;
            }
        }

        // assign the i-th observation with the nearest prediction
        //particles[nearest_i].associations.push_back();
    }
    cout << "Data Association Done" << endl;

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

    cout << "Update Weights ..." << endl;
    cout << "observations size = " << observations.size() << endl;
    vector<LandmarkObs> pred_m;

    for (int i=0; i<particles.size(); ++i) {
        /* TRANSFORM
         * transform the given sensor landmark observations from the VEHICLE'S coordinate system
        * into the MAP'S coodinate system, which the particles are located according to. */

        for (int n=0; n<observations.size(); ++n) {

            // transformed predicted particels[i] in map as pred_m[i]
            double trans_x = particles[i].x + (cos(particles[i].theta) * observations[n].x) - (sin(particles[i].theta) * observations[n].y);
            double trans_y = particles[i].y + (sin(particles[i].theta) * observations[n].x) + (cos(particles[i].theta) * observations[n].y);
            observations[n].x = trans_x;
            observations[n].y = trans_y;
            //cout << "x = " << particles[i].x << " --> x_m = " << pred_m[i] << endl;
            //cout << "y = " << particles[i].y << " --> y_m = " << pred_m[i] << endl;
        }
        cout << "Transformation Done" << endl;

        /* ASSOCIATION
         * associating the transformed observations(in MAP'S coordinate system)
         * with the nearest landmark on the map. */
        cout << "map_landmarks size = " << map_landmarks.landmark_list.size() << endl;
        ParticleFilter::dataAssociation(particles[]; observations);
        /*for (int m=0; m < map_landmarks.landmark_list.size(); ++m) {
            for (int n=0; n < observations) {

            }
        }*/


    }

    /* UPDATE WEIGHTS*/


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
