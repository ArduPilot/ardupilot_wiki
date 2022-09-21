rem remove any existing packages that may cause conflicts
pip uninstall -y sphinx lxml sphinx-rtd-theme sphinxcontrib-youtube beautifulsoup4

rem Install required python packages
pip install --user --upgrade -r requirements.txt

echo "Setup completed successfully!"
pause
