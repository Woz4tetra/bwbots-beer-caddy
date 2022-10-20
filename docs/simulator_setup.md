# Overview

This repo contains the Unity simulation and ROS packages necessary for simulating BWBots robots.

# Requirements

- Ubuntu 20.04+
- Unity 2021.3.11f1

# Unity Installation

The preferred way to install the Unity editor is through the **Unity Hub** application. To install:

```
sudo sh -c 'echo "deb https://hub.unity3d.com/linux/repos/deb stable main" > /etc/apt/sources.list.d/unityhub.list'
wget -qO - https://hub.unity3d.com/linux/keys/public | sudo tee /etc/apt/trusted.gpg.d/unityhub.asc
sudo apt update
sudo apt install unityhub
```

Then use the Unity Hub app to install a license:

1. Select the settings gear icon in the upper left next to your account icon
2. Go to **Licenses**
3. Select "Add" and choose the free personal license

After installing and activating your license, in the Unity Hub app:

1. Switch to the **Installs** tab
2. Select "Install Editor"
3. Find version **2021.3.11f1** and install

# Project Setup

## Opening the project

Once the correct version of the Unity Editor is setup, you'll need to configure the editor.

1. In the Unity Hub app's **Projects** tab
2. Select "Open" and find the root directory for the Unity project (`../unity/BWBots ROS1 Simulator`)

The Unity Hub app should automatically select the correct version for this project but if you a message about a version mismatch:

1. In the Unity Hub app
2. Find the BWBots ROS1 Simulator project in the Projects list
3. In the **Editor Version** column, select the correct editor version (`2021.3.11f1`)

If you don't see that version, you must install it locally using the steps above.

# IDE setup

Based on https://code.visualstudio.com/docs/other/unity

## Install .NET
- Install dotnet command line tool: (from: https://learn.microsoft.com/en-us/dotnet/core/install/linux-ubuntu#2004)
```
wget https://packages.microsoft.com/config/ubuntu/20.04/packages-microsoft-prod.deb -O packages-microsoft-prod.deb
sudo dpkg -i packages-microsoft-prod.deb
rm packages-microsoft-prod.deb
sudo apt-get update && \
  sudo apt-get install -y dotnet-sdk-6.0 dotnet-runtime-6.0 &&
  sudo apt-get install -y aspnetcore-runtime-6.0
```
- Install [C# Extension](https://marketplace.visualstudio.com/items?itemName=ms-dotnettools.csharp) from the VS Code Marketplace.
- In the VS Code Settings editor `(Ctrl+,)`, uncheck the C# extension's Omnisharp: Use Modern Net setting (`"omnisharp.useModernNet": false`).

## Install mono

```
sudo apt install gnupg ca-certificates
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 3FA7E0328081BFF6A14DA29AA6A19B38D3D831EF
echo "deb https://download.mono-project.com/repo/ubuntu stable-focal main" | sudo tee /etc/apt/sources.list.d/mono-official-stable.list
sudo apt update
sudo apt install -y mono-devel
```

## Setup Unity

- Open up Unity Preferences, External Tools, then browse for the Visual Studio Code executable as External Script Editor.

![alt text](https://code.visualstudio.com/assets/docs/other/unity/Unity_Preferences_External_Script_Editor.gif "Unity_Preferences_External_Script_Editor")

- Click "Regenerate project files"
