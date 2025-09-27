# LittleBot Documentation

This directory contains the complete documentation for the LittleBot ROS2 robotics package, built with Sphinx and using the Read the Docs theme.

## Quick Start

### Build Documentation

The easiest way to build the documentation is using the provided build script:

```bash
cd /path/to/littlebot/docs
./build_docs.sh
```

This script will:
1. Create a Python virtual environment
2. Install all necessary dependencies
3. Generate C++ API documentation with Doxygen
4. Build the Sphinx documentation
5. Optionally serve it locally

### Manual Build

If you prefer to build manually:

```bash
# Install dependencies
pip install sphinx sphinx-rtd-theme sphinx-autoapi breathe myst-parser

# Install system dependencies
sudo apt install doxygen graphviz

# Generate C++ documentation (optional)
doxygen Doxyfile

# Build HTML documentation
make html

# Serve locally (optional)
cd build/html && python3 -m http.server 8000
```

### View Documentation

After building, open `build/html/index.html` in your web browser, or visit `http://localhost:8000` if serving locally.

## Documentation Structure

```
docs/
├── source/                 # Source files for documentation
│   ├── _static/           # Static assets (CSS, images)
│   ├── _templates/        # Custom templates
│   ├── api/               # API reference documentation
│   ├── packages/          # Individual package documentation
│   ├── tutorials/         # Step-by-step tutorials
│   ├── conf.py            # Sphinx configuration
│   ├── index.rst          # Main documentation page
│   ├── installation.rst   # Installation guide
│   ├── getting_started.rst # Getting started guide
│   ├── troubleshooting.rst # Troubleshooting guide
│   ├── contributing.rst   # Contributing guidelines
│   └── changelog.rst      # Change log
├── build/                 # Generated documentation (HTML)
├── doxygen/               # Generated C++ API docs
├── Doxyfile              # Doxygen configuration
├── Makefile              # Build commands
├── build_docs.sh         # Automated build script
└── README.md             # This file
```

## Features

- **Read the Docs Theme**: Professional, responsive design
- **C++ API Documentation**: Auto-generated from code comments using Doxygen
- **Python API Documentation**: Auto-generated from docstrings using Sphinx autodoc
- **Markdown Support**: Write documentation in Markdown or reStructuredText
- **Cross-references**: Link between different parts of the documentation
- **Code Highlighting**: Syntax highlighting for multiple languages
- **Search Functionality**: Full-text search across all documentation
- **Mobile Responsive**: Works well on all devices

## Configuration

### Sphinx Configuration

The main configuration is in `source/conf.py`. Key settings:

- **Theme**: `sphinx_rtd_theme` (Read the Docs theme)
- **Extensions**: Autodoc, Napoleon, Intersphinx, Breathe, MyST
- **API Documentation**: Automatic generation from code comments
- **Cross-references**: Links to ROS2 and Python documentation

### Doxygen Configuration

C++ API documentation is configured in `Doxyfile`:

- **Input directories**: Automatically scans C++ source files
- **Output format**: XML (for Breathe integration) and HTML
- **Filtering**: Excludes build artifacts and generated files

### Custom Styling

Additional styling is defined in `source/_static/custom.css`:

- **Code blocks**: Enhanced syntax highlighting
- **Admonitions**: Styled note, warning, and tip boxes  
- **API documentation**: Improved formatting for function/class docs
- **ROS-specific**: Special styling for topics, nodes, and packages

## Writing Documentation

### Adding New Pages

1. Create a new `.rst` or `.md` file in the appropriate directory
2. Add the file to the relevant `toctree` directive
3. Rebuild the documentation

### API Documentation

C++ API documentation is automatically generated from code comments:

```cpp
/**
 * @brief Brief description of the function
 * @param param_name Description of parameter
 * @return Description of return value
 */
void myFunction(int param_name);
```

Python API documentation uses docstrings:

```python
def my_function(param_name):
    """Brief description of the function.
    
    Args:
        param_name: Description of parameter
        
    Returns:
        Description of return value
    """
    pass
```

### Cross-references

Link to other parts of the documentation:

```rst
See :doc:`installation` for setup instructions.
See :ref:`api-reference` for API details.
```

### Code Examples

Include code examples with syntax highlighting:

```rst
.. code-block:: bash

   ros2 launch littlebot_bringup littlebot_bringup.launch.py

.. code-block:: cpp

   #include <rclcpp/rclcpp.hpp>
   
   int main() {
       rclcpp::init(argc, argv);
       return 0;
   }
```

## Deployment

### Local Development

For local development, use the build script or manual build process described above.

### Read the Docs

To host on Read the Docs:

1. Connect your GitHub repository to Read the Docs
2. Configure the build in the Read the Docs dashboard
3. Set the documentation path to `/docs/`
4. Use Python 3.8+ and install requirements from `requirements.txt`

### GitHub Pages

To deploy to GitHub Pages:

1. Build the documentation locally
2. Copy `build/html/` contents to a `gh-pages` branch
3. Enable GitHub Pages in repository settings

## Troubleshooting

### Common Issues

**Build Errors:**
- Ensure all dependencies are installed
- Check that Doxygen is available if using C++ documentation
- Verify file paths in configuration files

**Missing API Documentation:**
- Check that source files contain proper documentation comments
- Verify include paths in Doxygen configuration
- Ensure Python modules are importable

**Theme Issues:**
- Confirm `sphinx-rtd-theme` is installed
- Check theme configuration in `conf.py`
- Clear the build directory and rebuild

### Getting Help

- Check the [Sphinx documentation](https://www.sphinx-doc.org/)
- Review [Read the Docs theme docs](https://sphinx-rtd-theme.readthedocs.io/)
- See [Doxygen manual](https://www.doxygen.nl/manual/) for C++ documentation
- Ask questions in the LittleBot GitHub repository

## Contributing to Documentation

We welcome contributions to improve the documentation:

1. Follow the style guidelines in `contributing.rst`
2. Test your changes by building locally
3. Submit a pull request with your improvements

For detailed guidelines, see :doc:`contributing`.