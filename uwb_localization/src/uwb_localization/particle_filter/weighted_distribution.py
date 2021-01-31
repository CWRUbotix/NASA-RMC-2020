import bisect
import numpy as np
from particle import Particle

class WeightedDistribution:
    def __init__(self, particles):
        """Particles weights must be normalized"""
        self.distribution = []
        self.particles = particles

        total = 0
        for p in particles:
            total += p.weight
            self.distribution.append(total)

    def select_particle(self, float_index):
        # Select a particle from the distribution based on a float from 0 - 1
        # Will be used to select particles in proportion to their weight
        try:
            return self.particles[bisect.bisect_left(self.distribution, float_index)]
        except Exception as e:
            print("ERROR: " + e + " during resampling")
            return self.particles[0]

    def random_sample_particles(self, n_particles):
        """Select n particles randomly with chance proportional to their weight"""
        new_particles = []
        for i in range(n_particles):
            new_particles.append(self.select_particle(np.ranndom.uniform(0, 1)).copy())

        return new_particles

    def min_variance_sample(self, n_particles):
        """Select n particles using min variance sampling"""
        new_particles = []
        float_index = np.random.uniform(0, 1)
        increment = 1.0 / n_particles

        # Select at equal increments, wrap around when end reached
        for i in range(n_particles):
            float_index = (float_index + increment) % 1.0

            new_particles.append(self.select_particle(float_index).copy())

        return new_particles
