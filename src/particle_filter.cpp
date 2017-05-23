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
#include <map>

#include "particle_filter.h"

using namespace std;

static default_random_engine gen;


void ParticleFilter::init(double x, double y, double theta, double std[]) {

  num_particles = 500;

  weights.resize(num_particles);
  particles.resize(num_particles);

  normal_distribution < double > N_x_init(0, std[0]);
  normal_distribution < double > N_y_init(0, std[1]);
  normal_distribution < double > N_theta_init(0, std[2]);

  double n_x;
  double n_y;
  double n_theta;

  for (int i = 0; i < num_particles; i++) {
    weights[i] = 1.0;

    n_x = N_x_init(gen);
    n_y = N_y_init(gen);
    n_theta = N_theta_init(gen);

    particles[i] = Particle {
      i,
      x + n_x,
      y + n_y,
      theta + n_theta,
      weights[i]
    };
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  normal_distribution < double > N_x_init(0, std_pos[0]);
  normal_distribution < double > N_y_init(0, std_pos[1]);
  normal_distribution < double > N_theta_init(0, std_pos[2]);

  for (int i = 0; i < num_particles; i++) {

    double n_x = N_x_init(gen);
    double n_y = N_y_init(gen);
    double n_theta = N_theta_init(gen);

    // update heading
    particles[i].theta += yaw_rate * delta_t + n_theta;

    // move in the (noisy) commanded direction
    double dist_x = velocity * delta_t + n_x;
    particles[i].x += cos(particles[i].theta) * dist_x;

    double dist_y = velocity * delta_t + n_y;
    particles[i].y += sin(particles[i].theta) * dist_y;
  }
}

void ParticleFilter::dataAssociation(std::vector < LandmarkObs > predicted, std::vector < LandmarkObs > & observations) {
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
  std::vector < LandmarkObs > observations, Map map_landmarks) {
}

void ParticleFilter::resample() {
}

void ParticleFilter::write(std::string filename) {
  // You don't need to modify this file.
  std::ofstream dataFile;
  dataFile.open(filename, std::ios::app);

  for (int i = 0; i < num_particles; ++i) {
    dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
  }
  dataFile.close();
}
