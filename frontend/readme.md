This folder contains the homepage of the website www.ardupilot.org.
This is done with simple HTML/JS pages using bootstrap. The home page (index.html) uses some AJAX includes, so some browsers block loading.

## Testing
In order to test the HTML in your computer, there are some options, for example: 

- Run Chrome as "chrome --allow-file-access-from-file" (without quotes). 
- Enter the frontend folder and run "python3 -m http.server 5000" (without quotes). 

Then use the browser to access the index.html file. 

## Testing with docker
### Building the Docker image
To build the Docker image, navigate to the directory containing the Dockerfile and execute the following command:
`docker build -t ardupilot_wiki_frontend .`

This will create a Docker image named ardupilot_wiki_frontend.

### Running the Docker container
Run the Docker container using the image created above and specify the desired port number:
`docker run -it --rm -p 8080:8080 -v "${PWD}:/frontend" ardupilot_wiki_frontend`

Default is port 8080. Replace with the port number you want the server to listen to. We are using a live volume, that allows to modify the file in the frontend directory and see the result on browser reloading.
For example:
`docker run -it --rm -e SERVICE_PORT=8080 -p 8080:8080 ardupilot_wiki_frontend`
With this command, the server will start and listen on port 8080.


## Generating WebP images

 Install cwebp utility and then :
`for file in images/*; do cwebp -resize 920 480 "$file" -o "${file%.*}.webp"; done`

Usage of Webp images : https://web.dev/serve-images-webp/
