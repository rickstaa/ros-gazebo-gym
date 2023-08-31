.. _envs:

Environments
============

Here, you can find all the **robot gymnasium environments** currently available. These environments within the robotics gymnasium encompass a diverse range
of task variations, allowing you to train your AI agents effectively.

.. toctree::
   :maxdepth: 4

   envs/panda/panda_env

.. note::

   Given that the :ros-gazebo-gym:`ROS Gazebo Gym <>` package adheres to the same structure as the :openai_ros:`openai_ros <>` package developed by `the construct`_, seamless 
   porting of environments from the openai_ros package to the :ros-gazebo-gym:`ROS Gazebo Gym <>` package is feasible. Should you be inclined to transition an environment
   from the :openai_ros:`openai_ros <>` package to the :ros-gazebo-gym:`ROS Gazebo Gym package <>`, we kindly request your review of the :ref:`add_new_env` and :ref:`contribution` sections, followed 
   by the submission of a pull request containing your ported environment.

.. _`the construct`: https://www.theconstructsim.com/
