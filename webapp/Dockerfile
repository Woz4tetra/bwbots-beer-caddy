FROM node:lts-alpine

# ---
# Configure
# ---

# Set the working directory to /opt inside the container
WORKDIR /opt/webapp

# Copy webapp files
COPY . .

# ---
# Build
# ---

# Install dependencies (npm ci makes sure the exact versions in the lockfile gets installed)
RUN npm ci

# Build the app
RUN npm run build

# ---
# Run
# ---

# Set the env to "production"
ENV NODE_ENV production

# Expose the port on which the app will be running (3000 is the default that `serve` uses)
EXPOSE 3000

# Start the app
CMD [ "npx", "serve", "build" ]
