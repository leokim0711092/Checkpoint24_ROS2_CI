#!/bin/bash

# Exit on any error. If something fails, there's no use
# proceeding further because Jenkins might not be able 
# to run in that case
set -e

# Directory where your workspace resides
WORKSPACE_DIR=~/webpage_ws

# Create workspace directory if it doesn't exist
mkdir -p "$WORKSPACE_DIR"

# Generate SSH keys if they don't exist
cd "$WORKSPACE_DIR"
if [ ! -f id_rsa ]; then
    ssh-keygen -q -N '' -C '' -f id_rsa
fi

if [ ! -f id_rsa_2 ]; then
    ssh-keygen -q -N '' -C '' -f id_rsa_2
fi

# Copy the SSH keys to the ~/.ssh directory
SSH_DIR=~/.ssh
mkdir -p "$SSH_DIR"
chmod 700 "$SSH_DIR"
cd "$SSH_DIR"

# Copy id_rsa if it doesn't exist
if [ ! -f id_rsa ]; then
    cp "$WORKSPACE_DIR/id_rsa" .
    cp "$WORKSPACE_DIR/id_rsa.pub" .
    chmod 600 id_rsa
    cat id_rsa.pub >> authorized_keys
    chmod 600 authorized_keys
fi

# Copy id_rsa_2 if it doesn't exist
if [ ! -f id_rsa_2 ]; then
    cp "$WORKSPACE_DIR/id_rsa_2" .
    cp "$WORKSPACE_DIR/id_rsa_2.pub" .
    chmod 600 id_rsa_2
    cat id_rsa_2.pub >> authorized_keys
    chmod 600 authorized_keys
fi

# Start the ssh agent
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_rsa
ssh-add ~/.ssh/id_rsa_2

# Create the config file in ~/.ssh directory
cat <<EOF > ~/.ssh/config
Host github.com-repo1
    HostName github.com
    User git
    IdentityFile ~/.ssh/id_rsa

Host github.com-repo2
    HostName github.com
    User git
    IdentityFile ~/.ssh/id_rsa_2
EOF

# Config git
git config --global user.name 'Jenkins Course'
git config --global user.email 'jenkins@theconstruct.ai'

