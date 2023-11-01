# Use an official Python runtime as a parent image
FROM python:3.11-slim

# Set the working directory in the container to /app
WORKDIR /frontend

COPY scripts/ /frontend/scripts/

ENV SERVICE_PORT 8080

# Make service's port available to the world outside this container
EXPOSE $SERVICE_PORT

# Install any needed packages specified in scripts/requirements.txt

RUN pip install --no-cache-dir -r scripts/requirements.txt

# Define our entrypoint script
ENTRYPOINT ["scripts/entrypoint.sh"]
