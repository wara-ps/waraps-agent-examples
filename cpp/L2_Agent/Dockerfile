FROM ubuntu:22.04
WORKDIR /app
COPY install_requirements.sh .

# Update the package list and install necessary packages
RUN chmod +x /app/install_requirements.sh && \
    /app/install_requirements.sh && \
# Clean up packages and cache to reduce image size
    apt-get clean && \
    apt-get remove -y cmake git libssl-dev && \
    apt-get autoremove -y
