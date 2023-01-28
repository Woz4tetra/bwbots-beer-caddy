FROM node:lts-alpine

RUN apk update && \
    apk add git tmux curl htop net-tools

# ---
# Configure
# ---

# Set the working directory to /opt inside the container
WORKDIR /opt/webapp

# Install app dependencies
# A wildcard is used to ensure both package.json AND package-lock.json are copied
# where available (npm@5+)
COPY package*.json ./

# ---
# Build
# ---

# Install dependencies (npm ci makes sure the exact versions in the lockfile gets installed)
RUN npm ci

# ---
# Run
# ---

# Copy webapp files
COPY . .

# Set the env to "production"
ENV NODE_ENV production

# Expose the port on which the app will be running (3000 is the default that `serve` uses)
EXPOSE 3000

# Start the app
CMD [ "npm", "start" ]
