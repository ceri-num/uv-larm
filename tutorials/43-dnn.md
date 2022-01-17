# Example of usage of Deep Neural Network.


Deep Neural Network, is certainlly the most magical and efficient technique in image recognition.
This technics is based on the learning of a huge number of weights attached to neurons (simple activation equations) connected together and organized in layers.

The learning porcess is achieved with un large number of examples.

The core difficulty in dnn is to propagate the knowledge over the overall weights and it is largelly relative to the structure of the layers in the neural network.

However, when an efficient strucure for a given task is correctly learned, the model can be shared and used with good confidances in the predictions.

An example of the capability of such a technique is provided with the [larm_dnn](https://bitbucket.org/imt-mobisyst/mb6-tbot/src/master/tutorials/larm_dnn/) example in tbot repo.

The example is almost ROS compatible...

First be sure you have a web cam on your coputer then:

```bash
rosrun larm_dnn apply-dnn.py
```