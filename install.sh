#!/bin/bash
set -e

mkdir -p ../assets

# Check if git-lfs is installed, if not, install it
if ! command -v git-lfs &> /dev/null; then
  echo "git-lfs not found, install it."
  curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
  sudo apt-get install git-lfs -y
  git lfs install
else
  echo "git-lfs is already installed."
fi

# Download USD file from Hugging Face Hub
echo "Downloading USD file from Hugging Face Hub..."

if [ -d "assets/melon/.git" ]; then
  echo "Repository already exists. Pulling latest changes..."
  git -C ../assets/melon pull
else
  git clone https://huggingface.co/datasets/SSatoya/melon_usd ../assets/melon
fi

echo "Download complete! Files are in '../assets/melon'."