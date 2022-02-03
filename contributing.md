# Contributing

When contributing to this repository, please first discuss the change you wish to make via issue,
email, or any other method with the owners of this repository before making a change.

Please note we have a code of conduct, please follow it in all your interactions with the project.

## Find projects to work on

Projects that are open for contributions can be found on the [issue tab](https://github.com/rickstaa/ros-gazebo-gym/) and will be labelled with the `help wanted` tag. Projects with the `beginner` label are ideal for beginning programmers. If you find a project that spikes your interest, leave a comment. We will then assign you to this project. After you did one project, you will be added as a contributor after which you can attach yourself to projects to work on. You can also submit a feature you would like to work on yourself by by opening [an issue](https://github.com/rickstaa/ros-gazebo-gym/issues/new).

## Forking process

1.  Fork the main [repository](https://github.com/rickstaa/ros-gazebo-gym/)
2.  Create your feature branch `git checkout -b feature/fooBar`
3.  Commit your changes `git commit -am 'Add some fooBar'`
4.  Push to the branch `git push origin feature/fooBar`
5.  Create a new Pull Request

## Commit instructions

We use husky pre-commit hooks to ensure code quality. To enable these hooks please:

1.  Install [node](https://nodejs.org/en/download/package-manager/).
2.  Install [python](https://www.python.org/downloads/).
3.  Run `npm install -D` and `pip install .[dev]` to install husky and the required linters.
4.  Run `npm run prepare` to setup the pre-commit hooks.

## Pull Request Process

1.  Ensure any install or build dependencies are removed before the end of the layer when doing a build.
2.  Update the README.md with details of changes to the interface, this includes new environment variables, exposed ports, useful file locations and container parameters.
3.  Increase the version numbers in any examples files and the README.md to the new version that this Pull Request would represent. The versioning scheme we use is [SemVer](http://semver.org/). To prevent errors you are recommended to use the [standard-version](https://github.com/conventional-changelog/standard-version) tool.
4.  Update the `CHANGELOG.md`. This is also done using the [standard-version](https://github.com/conventional-changelog/standard-version) tool.
5.  You may merge the Pull Request in once you have the sign-off of two other developers, or if you do not have permission to do that, you may request the second reviewer to merge it for you.

### Versioning instructions

We use the [standard-version](https://github.com/conventional-changelog/standard-version) package for versioning. This tool will automatically increase the version and create a changelog by parsing your commits using the [conventional Commits](https://conventionalcommits.org/). This package can be invoked using the `npm run release` command. After you cut a release, you can push the new git tag using the `git push --follow-tags origin main` command.

### Linting instructions

We use [husky](https://github.com/typicode/husky) pre-commit hooks to perform some linting and formatting actions. These actions will automatically run when you create a commit. Apart from this also some linting actions are performed by github actions when you create a pull request.

## Code of Conduct

### Our Pledge

In the interest of fostering an open and welcoming environment, we as contributors and maintainers pledge to making participation in our project and our community a harassment-free experience for everyone, regardless of age, body size, disability, ethnicity, gender identity and expression, level of experience, nationality, personal appearance, race, religion, or sexual identity and orientation.

### Our Standards

Examples of behaviour that contributes to creating a positive environment include:

*   Using welcoming and inclusive language
*   Being respectful of differing viewpoints and experiences
*   Gracefully accepting constructive criticism
*   Focusing on what is best for the community
*   Showing empathy towards other community members

Examples of unacceptable behaviour by participants include:

*   The use of sexualized language or imagery and unwelcome sexual attention or advances
*   Trolling, insulting/derogatory comments, and personal or political attacks
*   Public or private harassment
*   Publishing others' private information, such as a physical or electronic address, without explicit permission
*   Other conduct which could reasonably be considered inappropriate in a professional setting

### Our Responsibilities

Project maintainers are responsible for clarifying the standards of acceptable behaviour and are expected to take appropriate and fair corrective action in response to any instances of unacceptable behaviour.

Project maintainers have the right and responsibility to remove, edit, or reject comments, commits, code, wiki edits, issues, and other contributions that are not aligned to this Code of Conduct, or to ban temporarily or permanently any contributor for other behaviours that they deem inappropriate, threatening, offensive, or harmful.

### Scope

This Code of Conduct applies both within project spaces and in public spaces when an individual is representing the project or its community. Examples of representing a project or community include using an official project e-mail address posting via an official social media account, or acting as an appointed representative at an online or offline event. Representation of a project may be further defined and clarified by project maintainers.

### Attribution

This Code of Conduct is adapted from the [Contributor Covenant][homepage], version 1.4, available at [http://contributor-covenant.org/version/1/4][version]

[homepage]: http://contributor-covenant.org

[version]: http://contributor-covenant.org/version/1/4/
