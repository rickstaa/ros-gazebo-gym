# OpenAI ROS

OpenAI provides a complete Reinforcement Learning set of libraries that allow training software agents on tasks to learn by themselves how to do the task best. The main agents are software agents, like this example where [the OpenAI team trained an agent to play Dota 2](https://blog.openai.com/dota-2/).

One of the best tools of the OpenAI set of libraries is [the Gym](https://github.com/openai/gym). The Gym allows comparing Reinforcement Learning algorithms by providing a common ground called the Environments.

Unfortunately, even if the Gym allows to train robots, it does not provide environments to train ROS based robots using Gazebo simulations.

The [openai\_ros](http://wiki.ros.org/openai_ros) package wraps the Gym such that we can use it to train ROS robots in [Gazebo simulations](http://gazebosim.org/).

## How to use

Examples on how to use the [task environments](https://theconstructcore.bitbucket.io/openai_ros/index.html) contained in the  [openai\_ros](http://wiki.ros.org/openai_ros) package can be found in the [openai\_ros\_example](https://bitbucket.org/theconstructcore/openai_examples_projects/src/master/) repository.

## Documentation

See openai\_ros [docs](http://wiki.ros.org/openai_ros) for more in depth installation and usage instructions.

## API

The code API for this package can be found [here](https://theconstructcore.bitbucket.io/openai_ros/index.html).
