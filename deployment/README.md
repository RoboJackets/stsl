# STSL Deployment

The deployment process happens in two steps:
1. Debian Package Generation

   Debian packages are built from source code in a Docker container and saved into package_generation/debians.tar.gz.

2. Apt Repository Publication

   Debian packages are then added to an Aptly repository and published to an Azure cloud storage service.

## Prerequisites

Before running the deployment process, there are a few tools you'll need to install.

### Aptly

[Aptly](https://www.aptly.info/) is a tool for managing apt package repositories. Version 1.4 or newer is recommended for gpg 2 compatibility.

Recommended installation instructions:

1. Get GPG Key

   ```bash
   wget -qO - https://www.aptly.info/pubkey.txt | sudo apt-key add -
   ```

1. Add aptly's apt repo

   ```bash
   sudo apt-add-repository "deb http://repo.aptly.info/ squeeze main"
   ```

1. Install aptly

   ```bash
   sudo apt install aptly
   ```

**Note:** While aptly is available through the Ubuntu archives, the included version (as of 20.04) is Aptly 1.3, which is not compatible with gpg 2.

### Docker

[Docker](https://www.docker.com/) is a tool for containerization used to create a virtual environment for package generation.

Install Docker following their [installation instructions](https://docs.docker.com/engine/install/ubuntu/).

### Azure CLI

The Azure CLI allows for manipulating [Azure](https://azure.microsoft.com/) cloud resources from the command line.

Install the Azure CLI following their [installation instructions](https://docs.microsoft.com/en-us/cli/azure/install-azure-cli-linux?pivots=apt).

## Deploying STSL

**WARNING:** This process will push new packages to the publicly accessible apt repository. When this process completes, students will receive the updated version of the packages when they next update their system packages. Please, don't publish broken / test packages.

**NOTE:** Before you'll be able to upload to the Azure storage account, you'll need to be logged in via the Azure CLI with an account that has blob storage management permissions. You can login by running `az login`.

To run the full deployment, simply run:

```bash
./deploy.sh
```
