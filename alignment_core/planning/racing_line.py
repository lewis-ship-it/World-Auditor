import numpy as np
import random


class RacingLineOptimizer:

    def __init__(self, population=40, generations=60):

        self.population = population
        self.generations = generations


    def fitness(self, path):

        curvature = np.sum(np.abs(np.diff(path, axis=0)))

        length = np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))

        return length + curvature


    def mutate(self, path):

        new = path.copy()

        idx = random.randint(1, len(path)-2)

        new[idx] += np.random.normal(0, 2, size=2)

        return new


    def optimize(self, centerline):

        population = []

        for _ in range(self.population):

            noise = np.random.normal(0,1,centerline.shape)

            population.append(centerline + noise)

        for _ in range(self.generations):

            scores = [(self.fitness(p), p) for p in population]

            scores.sort(key=lambda x: x[0])

            population = [p for _,p in scores[:10]]

            while len(population) < self.population:

                parent = random.choice(population)

                population.append(self.mutate(parent))

        best = min(population, key=self.fitness)

        return best