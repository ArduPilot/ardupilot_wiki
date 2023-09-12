rem remove any existing packages that may cause conflicts
pip uninstall -y sphinx lxml sphinx-rtd-theme sphinxcontrib-youtube beautifulsoup4

rem Install sphinx
pip install --upgrade sphinx==7.1.2 "docutils<0.19" requests>=2.31.0

rem lxml for parameter parsing:
pip install --upgrade lxml

rem Install sphinx theme from ArduPilot repository
pip install --upgrade git+https://github.com/ArduPilot/sphinx_rtd_theme.git

rem and a youtube plugin:
pip install --upgrade git+https://github.com/ArduPilot/sphinxcontrib-youtube.git

echo "Setup completed successfully!"
pause
