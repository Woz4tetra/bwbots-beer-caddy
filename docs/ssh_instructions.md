# Generate ssh key-pair

Generate an SSH key

`ssh-keygen`

```(bash)
Generating public/private rsa key pair.
Enter file in which to save the key (~/.ssh/id_rsa): ~/.ssh/robeert
Created directory '~/.ssh'.
Enter passphrase (empty for no passphrase):
Enter same passphrase again:
Your identification has been saved in ~/.ssh/robeert.
Your public key has been saved in ~/.ssh/robeert.pub.
The key fingerprint is:
SHA256:-----------------------/------------------- nvidia@$robeert
The key's randomart image is:
```

# Setup SSH key directories
Copy `~/.ssh/robeert.pub` to `~/.ssh/authorized_keys`

`cp ~/.ssh/robeert.pub ~/.ssh/authorized_keys`


Copy `robeert` and `robeert.pub` to your local machine's `~/.ssh` directory.

Change permissions of these files to 600: `chmod 600 robeert*`

Test the log in: `ssh -i ~/.ssh/robeert nvidia@robeert.local`

# Disable password login (optional)

`sudo nano /etc/ssh/sshd_config`

Search for `#PasswordAuthentication yes`

Change to `PasswordAuthentication no`

Save and restart the ssh service:

`sudo service ssh restart`
