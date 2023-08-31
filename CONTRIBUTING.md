# Contribution guidelines

<!--alex ignore easy-->

We love your input! 🚀 We want to make contributing to this project as easy and transparent as possible, whether it's:

*   [Reporting a bug](https://github.com/rickstaa/ros-gazebo-gym/issues).
*   [Discussing the current state of the code](https://github.com/rickstaa/ros-gazebo-gym/discussions).
*   [Submitting a fix](https://github.com/rickstaa/ros-gazebo-gym/pulls).
*   [Proposing new features](https://github.com/rickstaa/ros-gazebo-gym/issues).
*   Becoming a maintainer.

## We Develop with Github

<!--alex ignore host-hostess-->

We use Github to host code, track issues and feature requests, and accept pull requests.

### We Use [Github Flow](https://docs.github.com/en), So All Code Changes Happen Through Pull Requests

Pull requests are the best way to propose changes to the codebase (we use [Github Flow](https://docs.github.com/en/get-started/quickstart/github-flow)). We actively welcome your pull requests:

1.  Fork the repo and create your branch from `main`.
2.  Add tests if you've added code that should be tested.
3.  If you've changed APIs, update the documentation.
4.  If you changed the documentation, please ensure it builds (see [Documentation guidelines](#documentation-guidelines)).
5.  Make sure your code lints.
6.  Commit your changes.
7.  Create a pull request to pull the changes of your development branch onto the `main` branch.
8.  Ensure that all the [pull request checks](https://github.com/rickstaa/ros-gazebo-gym/actions) were successful.

### Report bugs using Github's [issues](https://github.com/rickstaa/ros-gazebo-gym/issues)

<!--alex ignore easy-->

We use GitHub issues to track public bugs. Report a bug by [opening a new issue](https://github.com/rickstaa/ros-gazebo-gym/issues/new/choose); it's that easy!

#### Write bug reports with detail, background, and sample code

[This is an example](https://stackoverflow.com/q/12488905/180626) of a bug report, and I think it's a good model. Here's [another example from Craig Hockenberry](http://www.openradar.me/11905408), an app developer greatly respected in the community.

**Great Bug Reports** tend to have:

*   A quick summary and/or background.
*   Steps to reproduce:
    *   Be specific!
    *   Give sample code if you can. [A StackOverflow question](https://stackoverflow.com/q/12488905/180626) includes sample code that *anyone* with a base R setup can run to reproduce the error.
*   What you expected would happen
*   What happens?
*   Notes (possibly including why you think this might be happening or stuff you tried that didn't work).

People *love* thorough bug reports. I'm not even kidding.

## Write descriptive commit messages

Ensure you add an excellent descriptive commit message to this repository. A good guide can be found [here](https://www.conventionalcommits.org/en/v1.0.0/). When writing commit messages, please follow the [conventional commits specifications](https://www.conventionalcommits.org/en/v1.0.0/). Additionally, you are recommended to check out the [commitizen](https://github.com/commitizen/cz-cli) npm package, as it is beneficial in helping you write good commit messages.

## Use a Consistent coding Style

We use several language guidelines to increase code quality, readability, and usability.

### Python guidelines

<!--alex ignore black -->

*   **Linting:** Please ensure your Python code doesn't contain any errors by checking it with the [flake8 python linter](https://flake8.pycqa.org/en/latest/).
*   **Formatting:** Please format all your scripts using the [black python formatter](https://github.com/psf/black).

### Markdown guidelines

*   **Linting and formatting:** Please ensure your markdown code contains no errors and is formatted according to the [remark-lint](https://github.com/remarkjs/remark-lint) style guidelines.

## GitHub actions

The [ROS Gazebo Gym](https://github.com/rickstaa/ros-gazebo-gym) package contains several [GitHub actions](https://github.com/rickstaa/ros-gazebo-gym/actions), which check code changes against the language guidelines above. As a result, when the above guidelines are not met, you will receive an **error/warning** when you create a pull request. Some of these actions will make pull requests which you can use to fix some of these violations. For other **errors/warnings**, you are expected to handle them yourself before merging them into the main branch. If you think a coding guideline needs to be corrected or your code structure doesn't allow you to respect the guideline, please state so in the pull request.

## Pre-commit hooks

The [ROS Gazebo Gym](https://github.com/rickstaa/ros-gazebo-gym) package also contains several pre-commit hooks that enforce the guidelines above before committing. To enable these hooks, please:

1.  Install [node](https://nodejs.org/en/download/package-manager).
2.  Install [python](https://www.python.org/downloads).
3.  Run `npm install .` and `pip install -r requirements/dev_requirements.txt` to install husky and the required linters.

## Release guidelines

Releases are handled automatically by the [release-please-action](https://github.com/google-github-actions/release-please-action). This action uses the [release-please](https://github.com/googleapis/release-please) tool to create release pull requests based on the commits automatically. When writing commit messages, you must follow the [Commit guidelines](#write-descriptive-commit-messages) since this expects you to adhere to the [conventional commits specifications](https://www.conventionalcommits.org/en/v1.0.0/). For more information on how this tool works, see [the release-please documentation](https://github.com/googleapis/release-please).

## Documentation guidelines

See the [documentation guidelines](https://rickstaa.dev/ros-gazebo-gym/dev/doc_dev.html) for more information on how to contribute to the documentation.

## Any contributions you make will be under the MIT Software License

In short, when you submit code changes, your submissions are understood to be under the same [MIT License](https://choosealicense.com/licenses/mit/) that covers the project. Feel free to contact the maintainers if that's a concern.
