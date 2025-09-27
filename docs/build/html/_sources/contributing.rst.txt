Contributing
============

We welcome contributions to the LittleBot project! This document outlines how to contribute effectively.

Getting Started
---------------

1. **Fork the repository** on GitHub
2. **Clone your fork** locally:

   .. code-block:: bash

      git clone https://github.com/YOUR_USERNAME/littlebot.git
      cd littlebot

3. **Create a development branch**:

   .. code-block:: bash

      git checkout -b feature/your-feature-name

Development Guidelines
----------------------

Code Style
~~~~~~~~~~~

**C++ Code**:
- Follow the `ROS2 C++ Style Guide <https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html>`_
- Use ``clang-format`` for consistent formatting
- Include proper Doxygen documentation

**Python Code**:
- Follow PEP 8 style guidelines
- Use ``black`` for code formatting
- Include proper docstrings

**File Organization**:
- Place header files in ``include/package_name/``
- Place source files in ``src/``
- Place launch files in ``launch/``
- Place configuration files in ``config/``

Documentation
~~~~~~~~~~~~~

- Update relevant documentation for any changes
- Add inline comments for complex logic
- Update this Sphinx documentation as needed
- Include examples in your documentation

Testing
~~~~~~~

**Unit Tests**:
- Write unit tests for new functionality
- Place tests in ``test/`` directory
- Use Google Test for C++ tests
- Use pytest for Python tests

**Integration Tests**:
- Test with both real and simulated hardware
- Verify launch files work correctly
- Test ROS2 interfaces (topics, services, actions)

Commit Guidelines
-----------------

**Commit Messages**:
- Use the imperative mood ("Add feature" not "Added feature")
- Limit the first line to 72 characters
- Reference issues and pull requests when applicable

**Example**:

.. code-block:: text

   Add serial communication timeout handling

   - Implement configurable timeout for serial operations
   - Add retry logic for failed communications
   - Update documentation with timeout parameters
   
   Fixes #123

**Commit Types**:
- ``feat``: New feature
- ``fix``: Bug fix
- ``docs``: Documentation changes
- ``style``: Code style changes (formatting, etc.)
- ``refactor``: Code refactoring
- ``test``: Adding or updating tests
- ``chore``: Maintenance tasks

Pull Request Process
--------------------

1. **Ensure your branch is up to date**:

   .. code-block:: bash

      git checkout main
      git pull upstream main
      git checkout your-feature-branch
      git rebase main

2. **Run tests locally**:

   .. code-block:: bash

      colcon build
      colcon test
      colcon test-result --verbose

3. **Create a pull request** with:
   - Clear description of changes
   - Reference to related issues
   - Screenshots/videos if applicable
   - Test results

4. **Respond to review feedback** promptly

5. **Squash commits** if requested before merging

Areas for Contribution
----------------------

High Priority
~~~~~~~~~~~~~

- **Hardware drivers** for additional sensors
- **Navigation improvements** and new algorithms
- **Simulation enhancements** for better fidelity
- **Documentation** improvements and translations
- **Testing** coverage expansion

Medium Priority
~~~~~~~~~~~~~~~

- **RQT plugins** for additional functionality
- **Launch file** improvements and parameterization
- **Configuration tools** for easier setup
- **Performance optimizations**

Low Priority
~~~~~~~~~~~~

- **Additional examples** and tutorials
- **Visualization improvements**
- **Code cleanup** and refactoring
- **Build system** improvements

Reporting Issues
----------------

When reporting bugs or requesting features:

1. **Check existing issues** to avoid duplicates
2. **Use the issue templates** provided
3. **Include system information**:
   - Operating system and version
   - ROS2 distribution
   - Hardware specifications (if relevant)
4. **Provide minimal reproducible examples**
5. **Include relevant logs and error messages**

**Bug Report Template**:

.. code-block:: text

   **Describe the bug**
   A clear and concise description of what the bug is.

   **To Reproduce**
   Steps to reproduce the behavior:
   1. Go to '...'
   2. Click on '....'
   3. Scroll down to '....'
   4. See error

   **Expected behavior**
   A clear and concise description of what you expected to happen.

   **Environment**
   - OS: [e.g. Ubuntu 22.04]
   - ROS2 Version: [e.g. Humble]
   - Package Version: [e.g. 1.0.0]

Code Review Process
-------------------

All contributions go through code review:

**Reviewers will check**:
- Code quality and style compliance
- Test coverage and functionality
- Documentation completeness
- Integration with existing code
- Performance implications

**Timeline**:
- Initial review within 1 week
- Follow-up reviews within 3 business days
- Merging after approval from 2 maintainers

Release Process
---------------

LittleBot follows semantic versioning (SemVer):

- **Major** (X.0.0): Breaking changes
- **Minor** (1.X.0): New features, backward compatible
- **Patch** (1.0.X): Bug fixes, backward compatible

**Release Cycle**:
- Major releases: As needed for significant changes
- Minor releases: Monthly or quarterly
- Patch releases: As needed for critical fixes

Community Guidelines
--------------------

**Be Respectful**:
- Use welcoming and inclusive language
- Respect differing viewpoints and experiences
- Give and accept constructive feedback

**Be Collaborative**:
- Help newcomers and answer questions
- Share knowledge and best practices
- Work together to improve the project

**Be Professional**:
- Focus on technical merit of contributions
- Avoid personal attacks or harassment
- Follow the ROS Community Code of Conduct

Recognition
-----------

Contributors are recognized in:
- Release notes and changelogs
- GitHub contributor statistics
- Documentation acknowledgments
- Community presentations and papers

Getting Help
------------

**Development Questions**:
- Create an issue with the "question" label
- Join the ROS Discourse forums
- Reach out to maintainers directly

**Real-time Help**:
- ROS Discord community
- Weekly community meetings (schedule TBD)

Thank you for contributing to LittleBot!