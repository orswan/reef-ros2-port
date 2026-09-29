# ROS 2 Jazzy & Gazebo Harmonic Docker Setup (noVNC for macOS)

Forwarding Gazebo 3D graphics through XQuartz over macOS X11 often results in OpenGL context failures. Using an internal virtual framebuffer (`Xvfb`) and `noVNC` sidesteps this by rendering the graphics via software inside the Linux container and streaming the 3D viewport directly to a web browser.

---

## 1. Daily Startup Workflow (New Session)

Follow these steps every time you restart your Mac or begin a new coding session.

### Step 1: Start Container & Open Shell (Mac Terminal)

Open your standard macOS terminal. Start your existing stopped container and attach an interactive shell:

```bash
docker start ros2_novnc_container
docker exec -it ros2_novnc_container bash

```

### Step 2: Start Virtual Display Services (Docker Terminal)

Inside the new Docker terminal you just opened, run the VNC server helper script (which you will create during the first-time setup):

```bash
~/start_vnc.sh

```

### Step 3: Connect Display Viewer (Mac Browser)

1. Open Safari, Chrome, or Firefox on your Mac.
2. Navigate to: `http://localhost:8080/vnc.html`
3. Click **Connect**. You will see a dark desktop canvas (the Fluxbox window manager).

### Step 4: Launch Gazebo Simulation (Docker Terminal)

Return to your attached Docker terminal. You can now launch Gazebo, and the graphical interface will pipe directly to your browser window:

```bash
gz sim shapes.sdf

```

Note: You can open additional parallel terminals at any time by running `docker exec -it ros2_novnc_container bash` in a new Mac terminal window.

---

## 2. First-Time Environment Setup

Run these commands **once** to create and configure the container environment.

### Step 1: Create Container with Port Forwarding (Mac Terminal)

In your macOS terminal, create a new container instance. This command binds port `8080` to allow browser access to the web stream, uses the `ros:jazzy` base image, and mounts your local Mac workspace folder into the container:

```bash
docker run -it --name ros2_novnc_container -p 8080:8080 -v ~/ros2_ws:/root/ros2_ws ros:jazzy

```

### Step 2: Install ROS Packages & VNC Dependencies (Docker Terminal)

Your terminal prompt will now be inside the Docker container. Install the necessary Gazebo packages and the web streaming display tools:

```bash
apt update && apt install -y \
  ros-jazzy-ros-gz \
  xvfb \
  x11vnc \
  novnc \
  fluxbox

```

### Step 3: Create VNC Startup Automation Script (Docker Terminal)

Still in the Docker terminal, run these commands to generate `~/start_vnc.sh` line-by-line so the background display services can be started easily in the future:

```bash
echo '#!/bin/bash' > ~/start_vnc.sh
echo 'Xvfb :99 -screen 0 1280x1024x24 &' >> ~/start_vnc.sh
echo 'export DISPLAY=:99' >> ~/start_vnc.sh
echo 'fluxbox &' >> ~/start_vnc.sh
echo 'x11vnc -display :99 -forever -shared -nopw -bg' >> ~/start_vnc.sh
echo 'websockify --web=/usr/share/novnc/ 8080 localhost:5900 &' >> ~/start_vnc.sh
echo 'echo "noVNC server active at http://localhost:8080/vnc.html"' >> ~/start_vnc.sh

chmod +x ~/start_vnc.sh

```

### Step 4: Configure Container Shell Environment (Docker Terminal)

Add persistent environment variables and ROS sourcing to your container's `.bashrc` file. This ensures every new Docker shell automatically loads your ROS 2 Jazzy environment and sets the necessary software rendering overrides:

```bash
echo "export DISPLAY=:99" >> ~/.bashrc
echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc
echo "export MESA_GL_VERSION_OVERRIDE=3.3" >> ~/.bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "if [ -f /root/ros2_ws/install/setup.bash ]; then source /root/ros2_ws/install/setup.bash; fi" >> ~/.bashrc
source ~/.bashrc

```

---

## 3. Quick Reference & Troubleshooting

| Symptom / Error | Cause | Solution |
| --- | --- | --- |
| `bash: gz: command not found` | Gazebo ROS integration packages missing or ROS base environment is unsourced. | In the Docker terminal, run `apt install ros-jazzy-ros-gz` and `source /opt/ros/jazzy/setup.bash`.

 |
| Browser page refuses to connect (`http://localhost:8080`) | Port `8080` is not exposed or the `websockify` process died. | Ensure the container was originally created with the `-p 8080:8080` flag. Re-run `~/start_vnc.sh` in the Docker terminal. |
| `OpenGL 3.3 is not supported` | Software renderer is advertising a legacy OpenGL profile. | Ensure `export MESA_GL_VERSION_OVERRIDE=3.3` is actively set in your current Docker terminal session. |