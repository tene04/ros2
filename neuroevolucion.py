import gymnasium as gym
import numpy as np
from MLP import MLP

class neuroevolution():
    # Hemos modificado el algoritmo genético dado en clase para que sea más eficiente y se adopte mejor a los requisitos del gym.
    # Tras probar varias variantes, hemos llegado a la conclusión que funciones como crossover y torneo eran más perjudiciales que 
    # beneficiosas, el primero porque no se adaptaba bien a la estructura de la red y hacía cambios muy bruscos y el segundo porque
    #  en este caso es mejor seleccionar a los mejores individuos y no hacer un torneo entre ellos. Por esta razón hemos partido de 
    # los mejores individuos de la generación anterior para crear la siguiente.

    def __init__(self, input, hidden, output, fitness_func, elite_fraction, pmut, N=100):
        self.input = input # tamaño de la entrada
        self.hidden = hidden # tamaño de la capa oculta, por simplicidad solo hay una
        self.output = output # tamaño de la salida
        self.fitness_func = fitness_func # función de fitness
        self.elite_fraction = elite_fraction # porcentaje de élite que se usará para la reproducción	
        self.pmut = pmut # probabilidad de mutación
        self.N = N # tamaño de la población
        self.poblation = self.create() # población inicial

    def create(self):
        # crea una población de N individuos
        poblation = []
        for _ in range(self.N):
            net = MLP(self.input, self.hidden, self.output)
            poblation.append(net)
        return poblation
    
    def mutate(self, weights):
        # muta los pesos de un individuo
        new_weights = weights.copy()
        for i in range(len(new_weights)):
            if np.random.rand() < self.pmut:
                new_weights[i] += np.random.normal(0, abs(new_weights[i]) * 0.5 + 1e-3) # se altera según el valor del peso
        return new_weights
    
    def evolution(self, ngen, trace, env_name="LunarLander-v3", aux=None):
        if aux is None:
            env = gym.make(env_name)
        else:
            env = gym.make(env_name, use_lidar=aux)
        
        for generation in range(ngen): # para cada generación
            # se evalúa la población y se queda con los mejores individuos
            fitness = [self.fitness_func(net, env, 5) for net in self.poblation]
            elite_count = int(self.elite_fraction * self.N)
            elite_indices = np.argsort(fitness)[-elite_count:]
            elites = [self.poblation[i] for i in elite_indices]

            if generation % trace == 0:
                print(f"Generation {generation} | Max fitness: {max(fitness)} | Avg fitness: {np.mean(fitness)}")

            # con los mejores individuos se crea una nueva población
            new_pobla = []
            for _ in range(self.N):
                parent = np.random.choice(elites)
                child = MLP(self.input, self.hidden, self.output)
                child.set_weights(self.mutate(parent.get_weights()))
                new_pobla.append(child)

            self.poblation = new_pobla

        env.close()
        best_idx = np.argmax([self.fitness_func(net, env, 5) for net in self.poblation])
        best_network = self.poblation[best_idx]
        return best_network # se devuelve el mejor individuo de la última generación