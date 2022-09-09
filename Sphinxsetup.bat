rem remove any existing packages that may cause conflicts
pip uninstall -y sphinx lxml sphinx-rtd-theme sphinxcontrib-youtube beautifulsoup4

rem Install sphinx
pip install --upgrade sphinx==5.1.1 docutils==0.16

rem lxml for parameter parsing:
pip install --upgrade lxml

rem Install sphinx theme from ArduPilot repository
pip install --upgrade git+https://github.com/ArduPilot/sphinx_rtd_theme.git

rem and a youtube plugin:
pip install --upgrade git+https://github.com/ArduPilot/sphinxcontrib-youtube.git

rem and a parser to use getting posts from Discourse (forum) and insert in FrontEnd
pip install --upgrade beautifulsoup4

echo "Setup completed successfully!"
pause
