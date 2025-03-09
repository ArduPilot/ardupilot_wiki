rem remove any existing packages that may cause conflicts
python3 -m pip uninstall -y sphinx lxml sphinx-rtd-theme sphinxcontrib-youtube beautifulsoup4

rem Install required python packages
python3 -m pip install --user --upgrade -r requirements.txt

echo "Setup completed successfully!"
pause
