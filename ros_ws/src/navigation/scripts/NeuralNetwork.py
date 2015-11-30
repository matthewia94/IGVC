# A neural network implementation in python

import numpy as np


class NeuralNetwork:
    def __init__(self, num_inputs, num_hidden, num_outputs):
        # Seed the rng
        np.random.seed()

        # Set the number of nodes for each layer
        self.num_inputs = num_inputs
        self.num_hidden = num_hidden
        self.num_outputs = num_outputs

        # Random weights centered at 0 with std dev of 1
        self.ih_weights = np.matrix([2*np.random.random(self.num_hidden) - 1 for i in range(self.num_inputs)])
        self.ho_weights = np.matrix([2*np.random.random(self.num_outputs) - 1 for i in range(self.num_hidden)])

        self.input_layer = np.ones(self.num_inputs)
        self.hidden_layer = np.ones(self.num_hidden)
        self.output_layer = np.zeros(self.num_outputs)

        # Set the biases for the hidden and output layer to 1
        self.hidden_biases = np.ones(self.num_hidden)
        self.output_biases = np.ones(self.num_outputs)

        self.learn_rate = .01

    @staticmethod
    def sigmoid(x, deriv=False):
        if deriv:
            return x*(1-x)

        return 1/(1+np.exp(-x))

    def feedforward(self):
        # A1 to get array instead of 1xN matrix
        self.hidden_layer = (self.sigmoid(self.input_layer * self.ih_weights) + self.hidden_biases).A1
        self.output_layer = ((self.hidden_layer * self.ho_weights) + self.output_biases).A1

        # print self.input_layer
        # print self.hidden_layer
        # print self.output_layer

    def backpropagation(self):
        delta_hidden = self.sigmoid(self.output_layer, deriv=True) * self.error()
        ho_weights_update = self.ho_weights + self.learn_rate * delta_hidden * np.matrix(self.output_layer).transpose()
        delta_input = self.sigmoid(self.hidden_layer, deriv=True) * (delta_hidden*self.ho_weights.transpose()).A1
        self.ho_weights = ho_weights_update
        self.ih_weights += self.learn_rate * np.matrix(self.input_layer).transpose() * delta_input

    def error(self):
        return self.input_layer - self.output_layer

def main():
    n = NeuralNetwork(1, 2, 1)
    for i in range(10000):
        n.feedforward()
        n.backpropagation()
        print n.output_layer

if __name__ == "__main__":
    main()
