Welcome to littlebot's documentation!
====================================

Contents:

.. toctree::
   :maxdepth: 2
   :caption: Contents

   overview
   api
   cpp-doxygen


Getting started
---------------
This documentation set contains:

- A general overview (`overview.rst`).
- Python/autodoc-generated API pages (if you add Python modules) in `api.rst`.
- C++ API generated from Doxygen (via Breathe) in `cpp-doxygen.rst`.

Build locally
-------------
Install the python dependencies (recommended to use a virtualenv):

.. code-block:: bash

  python -m pip install -r docs/requirements.txt

Generate the Doxygen XML (the Read the Docs build runs Doxygen in a pre-build step):

.. code-block:: bash

  cd docs
  doxygen Doxyfile

Build with Sphinx:

.. code-block:: bash

  cd docs
  make html


Overview
--------
Create `docs/overview.rst` to add general project documentation.

API (placeholder)
------------------
Create `docs/api.rst` and add any python modules you want autodocumented.

Doxygen / C++ API
-----------------
The `cpp-doxygen.rst` page will include the Doxygen output via Breathe.
