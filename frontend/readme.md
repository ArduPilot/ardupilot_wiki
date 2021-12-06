This folder contains the homepage of the website www.ardupilot.org. This is done with simple HTML/JS pages using bootstrap. The home page (index.html) uses some AJAX includes, so some browsers block loading. 

In order to test the HTML in your computer, there are some options, for example: 

- Run Chrome as "chrome --allow-file-access-from-file" (without quotes). 
- Enter the frontend folder and run "python3 -m http.server 5000" (without quotes). 

Then use the browser to access the index.html file. 

TO-DO: 
How to create new pages 
How to edit the menu.

## Generating WebP images

 Install cwebp utility and then :
`for file in images/*; do cwebp -resize 920 480 "$file" -o "${file%.*}.webp"; done`

Usage of Webp images : https://web.dev/serve-images-webp/
