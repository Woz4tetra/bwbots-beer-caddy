# Laptop workstation setup

- Instructions for Ubuntu 20

## Generate SSH keys

- Generate identifier keys: `ssh-keygen`. No passphrase. Default file directory.

```
Generating public/private rsa key pair.
Enter file in which to save the key (~/.ssh/id_rsa):
Enter passphrase (empty for no passphrase):
The key fingerprint is:
SHA256:?? ??@??
The key's randomart image is:
??
```

## Setup git

- `sudo apt update`
- `sudo apt install -y git tmux htop xclip`
- Enter identification (replace with account the keys are associated with):

  - `git config --global user.email "your@email.com"`
  - `git config --global user.name "yourusername"`

- Run the following command:

```
cat > ~/.ssh/config << 'EOF'
host github.com
  HostName github.com
  IdentityFile ~/.ssh/id_rsa
  User git
EOF
```

- Log into https://github.com/
- Go to settings

![alt text](images/GithubMenu.jpg "GithubMenu")

- Navigate to SSH and GPG keys

![alt text](images/GithubSSHkeys.jpg "GithubSSHkeys")

- Copy the contents of the public key file `id_rsa.pub`:

  - `xclip -sel c < ~/.ssh/id_rsa.pub`

- Click `Add SSH key`

- Paste contents of `id_rsa.pub` here

![alt text](images/GithubAddKey.jpg "GithubAddKey")

- Click `Add SSH key`

- `cd ~`
- Clone repository: `git clone git@github.com:Woz4tetra/bwbots-beer-caddy.git --recursive`
- `cd ./bwbots-beer-caddy`
- `git config pull.rebase false`

# Connect to the robot via SSH

- Obtain the ssh keys `robeert` and `robeert.pub` (ping the repo authors)
- `mv robeert ~/.ssh/`
- `mv robeert.pub ~/.ssh/`
- Log in: `ssh -i ~/.ssh/robeert nvidia@<your IP>`

# Build workstation docker container

Recommended editor: https://code.visualstudio.com/download

- Install dev container extension: `code --install-extension ms-vscode-remote.remote-containers`

## Non-NVidia workstations

On your local machine, run this command:

- ~/bwbots-beer-caddy/docker/workstation/build_container

## NVidia workstations (for training neural nets)

On your local machine, run this command:

- ~/bwbots-beer-caddy/docker/workstation/build_training_container
