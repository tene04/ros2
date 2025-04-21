import numpy as np

class MLP:
    def __init__(self, input_size, hidden_sizes, output_size):
        # Tamaños de todas las capas, incluyendo entrada y salida
        layer_sizes = [input_size] + hidden_sizes + [output_size]

        # Inicializa pesos y sesgos para cada capa
        self.weights = []
        self.biases = []

        for i in range(len(layer_sizes) - 1):
            w = np.random.randn(layer_sizes[i], layer_sizes[i+1]) * 0.5
            b = np.zeros(layer_sizes[i+1])
            self.weights.append(w)
            self.biases.append(b)

    def relu(self, x):
        # función de activación Relu
        return np.maximum(0, x)

    def forward(self, x):
        # Propagación hacia adelante
        for i in range(len(self.weights) - 1):
            x = self.relu(np.dot(x, self.weights[i]) + self.biases[i])
        x = np.dot(x, self.weights[-1]) + self.biases[-1]  
        return x

    def get_weights(self):
        # devuelve los pesos y sesgos como un vector plano
        flat_weights = []
        for w, b in zip(self.weights, self.biases):
            flat_weights.append(w.flatten())
            flat_weights.append(b)
        return np.concatenate(flat_weights)

    def set_weights(self, weights):
        # establece los pesos y sesgos a partir de un vector plano
        idx = 0
        for i in range(len(self.weights)):
            w_shape = self.weights[i].shape
            b_shape = self.biases[i].shape

            w_size = np.prod(w_shape)
            b_size = np.prod(b_shape)

            self.weights[i] = weights[idx:idx+w_size].reshape(w_shape)
            idx += w_size
            self.biases[i] = weights[idx:idx+b_size].reshape(b_shape)
            idx += b_size

