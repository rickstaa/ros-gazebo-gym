===============
Release package
===============

.. contents:: Table of Contents

Code formatting guidelines
==========================

To increase code quality, readability and usability, we use several language guidelines.

Python guidelines:
    * **Linting:** Please ensure your python code doesn't contain any errors by checking it with the `flake8 python linter`_.
    * **Formatting:** Please format all your scripts using the `black python formatter`_.

.. _`flake8 python linter`: https://flake8.pycqa.org/en/latest/
.. _`black python formatter`: https://github.com/psf/black

Markdown guidelines:
    * **Linting and formatting:** Please make sure your markdown code contains no errors and is formatted according to the `remark-lint`_ style guidelines.

.. _`remark-lint`: https://github.com/remarkjs/remark-lint

.. note::
    The ROS Gazebo Gym framework contains several `GitHub actions`_, which check code changes
    against these coding guidelines. As a result, when the above guidelines are not met, you will
    receive an error/warning when you create a pull request. Some of these actions will create pull requests
    which you can use to fix some of these violations. For other errors/warning, you are expected to handle
    them yourself before merging them into the master branch. If you think a code guideline is not correct
    or your code structure doesn't allow you to respect the guideline, please state so in the
    pull request.

.. _`Github Actions`: https://github.com/rickstaa/ros-gazebo-gym/actions


Pre-commit hooks
----------------

The ROS Gazebo Gym package also contains several pre-commit hooks that can be used to enforce the aforementioned guidelines before a commit is made. To enable these hooks please:

1.  Install `node <https://nodejs.org/en/download/package-manager>`_.
2.  Install `python <https://www.python.org/downloads>`_.
3.  Run ``npm install -D`` and ``pip install .[dev]`` to install husky and the required linters.
4.  Run ``npm run prepare`` to setup the pre-commit hooks.


General guidelines
==================

Release guidelines
------------------

Before releasing the package, make sure the following steps are performed:

    #. Create a new branch on which you implement your changes.
    #. Commit your changes.
    #. Create a pull request to pull the changes of your development branch onto the master branch.
    #. Make sure that all the `pull request checks`_ were successful.
    #. Add a version label to (``bump:patch``, ``bump:minor`` or ``bump:major``) to the pull request.
    #. Squash and merge your branch with the noetic branch.
    #. Create a release using the GitHub draft release tool.

.. _`pull request checks`: https://github.com/rickstaa/ros-gazebo-gym/actions

Commit guidelines
-----------------

Make sure you add a good descriptive commit message while committing to this repository. A
good guide can be found `here`_. Please check out the `commitizen`_ npm package as it is very helpful
in helping you write good commit messages.

.. _`here`: https://www.conventionalcommits.org/en/v1.0.0/
.. _`commitizen`: https://github.com/commitizen/cz-cli

Versioning guidelines
---------------------

Additionally, please use the `versioning guidelines specified at semver.org <https://semver.org/>`_.
