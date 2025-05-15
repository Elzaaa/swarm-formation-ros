# Docker scripts
Here are some scripts that can make using this project easier.\

Install Docker:

```bash 
# It's preferably to use -n flag that enables Nvidia drivers support.
bash install_docker.sh -n # (Re)install Docker
bash build_docker.sh -n # Build Docker container:
bash run_docker.sh -n # Run Docker container
Ubuntu 22.04
ROS2 Version Humble 
Moveti2 
```

You can access the running container:
```bash
bash into_docker.sh
```

### 🚀 Quick Start with Scripts

1. **Clone this repository**  

2. **Install Docker** if needed:  
   \`\`\`bash
   bash docker/install_docker.sh -n
   \`\`\`

3. **Build Docker image**:  
   \`\`\`bash
   bash docker/build_docker.sh -n
   \`\`\`

4. **Run container with one command**:  
   \`\`\`bash
   ./docker/run_container.sh
   \`\`\`

5. *(Optional)* Run a second container for echoing or debugging:*  
   \`\`\`bash
   ./docker/run_echo.sh
   \`\`\`

