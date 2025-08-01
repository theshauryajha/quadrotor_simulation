#!/bin/bash

# Quadrotor Simulation Setup Script
# This script sets up the ROS workspace and environment for the quadrotor simulation

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Emojis for better visual feedback
CHECK="✅"
CROSS="❌"
WARNING="⚠️"
INFO="ℹ️"
ROCKET="🚀"
FOLDER="📁"
PACKAGE="📦"
HAMMER="🔨"
WRENCH="🔧"
WORLD="🌍"
INSTRUCTIONS="📋"
TAG="🏷️"

# Default ROS Noetic setup file path
ROS_SETUP="/opt/ros/noetic/setup.bash"

echo -e "${CYAN}${ROCKET} Quadrotor Simulation Setup${NC}"
echo -e "${CYAN}==============================${NC}"
echo ""

# Function to print colored status messages
print_status() {
    local emoji="$1"
    local message="$2"
    local color="$3"
    echo -e "${color}${emoji} ${message}${NC}"
}

print_error() {
    print_status "$CROSS" "$1" "$RED"
}

print_success() {
    print_status "$CHECK" "$1" "$GREEN"
}

print_warning() {
    print_status "$WARNING" "$1" "$YELLOW"
}

print_info() {
    print_status "$INFO" "$1" "$BLUE"
}

# Step 1: Check ROS Noetic
check_ros_noetic() {
    print_info "Checking for ROS Noetic installation..." "$BLUE"
    
    if [ -f "$ROS_SETUP" ]; then
        print_success "Found ROS Noetic setup at $ROS_SETUP"
        source "$ROS_SETUP"
        
        if [ "$ROS_DISTRO" = "noetic" ]; then
            print_success "ROS Noetic is installed and sourced successfully"
            return 0
        else
            print_warning "Sourced file, but ROS_DISTRO is '$ROS_DISTRO' (expected: noetic)"
            return 1
        fi
    else
        print_error "ROS Noetic not found at $ROS_SETUP"
        print_error "Please install ROS Noetic first: http://wiki.ros.org/noetic/Installation"
        return 1
    fi
}

# Step 2: Rename directory structure
setup_workspace_structure() {
    print_info "Setting up workspace structure..." "$BLUE"
    
    # Get current directory (should be catkin_ws/quadrotor_simulation)
    CURRENT_DIR=$(pwd)
    PARENT_DIR=$(dirname "$CURRENT_DIR")
    
    # Check if we're in the right location
    if [[ ! "$CURRENT_DIR" == */quadrotor_simulation ]]; then
        print_error "Please run this script from inside the quadrotor_simulation directory"
        return 1
    fi
    
    # Check if src directory already exists in parent
    if [ -d "$PARENT_DIR/src" ]; then
        print_warning "src directory already exists in workspace"
        echo "This will remove the existing src directory and replace it."
        read -p "Continue? (y/N): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            print_error "Setup cancelled by user"
            return 1
        fi
        rm -rf "$PARENT_DIR/src"
    fi
    
    # Rename current directory to src
    print_info "Renaming quadrotor_simulation to src..."
    cd "$PARENT_DIR"
    mv quadrotor_simulation src
    
    if [ $? -eq 0 ]; then
        print_success "Workspace structure set up successfully"
        cd src
        return 0
    else
        print_error "Failed to rename directory"
        return 1
    fi
}

# Step 3: Check rosdep
check_rosdep() {
    print_info "Checking rosdep installation..." "$BLUE"
    
    if command -v rosdep >/dev/null 2>&1; then
        print_success "rosdep found"
        
        # Initialize rosdep if not already done
        if [ ! -d "/etc/ros/rosdep" ]; then
            print_info "Initializing rosdep..."
            sudo rosdep init
            rosdep update
        fi
        return 0
    else
        print_error "rosdep not found"
        print_error "Please install with: sudo apt install python3-rosdep"
        return 1
    fi
}

# Step 4: Check and install AprilTag ROS
check_install_apriltag_ros() {
    print_info "Checking for AprilTag ROS package..." "$BLUE"
    
    # Check if apriltag_ros package is available
    if rospack find apriltag_ros >/dev/null 2>&1; then
        print_success "AprilTag ROS package found"
        return 0
    else
        print_warning "AprilTag ROS package not found"
        print_info "Installing AprilTag ROS..."
        
        # Try to install via apt first
        if sudo apt update && sudo apt install -y ros-noetic-apriltag-ros; then
            print_success "AprilTag ROS installed successfully via apt"
            
            # Source ROS setup again to make sure new package is available
            source "$ROS_SETUP"
            
            # Verify installation
            if rospack find apriltag_ros >/dev/null 2>&1; then
                print_success "AprilTag ROS installation verified"
                return 0
            else
                print_warning "AprilTag ROS installed but not found in ROS path"
                print_info "This might be resolved after workspace build"
                return 0
            fi
        else
            print_error "Failed to install AprilTag ROS via apt"
            print_info "Will attempt to build from source in workspace..."
            
            # Move to workspace src directory
            cd ..
            
            # Clone apriltag_ros repository
            print_info "Cloning AprilTag ROS from GitHub..."
            if git clone https://github.com/AprilRobotics/apriltag_ros.git src/apriltag_ros; then
                print_success "AprilTag ROS source code downloaded"
                
                # Also clone apriltag library if needed
                if [ ! -d "src/apriltag" ]; then
                    print_info "Cloning AprilTag library..."
                    if git clone https://github.com/AprilRobotics/apriltag.git src/apriltag; then
                        print_success "AprilTag library downloaded"
                    else
                        print_warning "Failed to clone AprilTag library, but continuing..."
                    fi
                fi
                
                cd src
                return 0
            else
                print_error "Failed to clone AprilTag ROS repository"
                print_error "You may need to install it manually later"
                return 1
            fi
        fi
    fi
}

# Step 5: Install dependencies
install_dependencies() {
    print_info "Installing workspace dependencies..." "$BLUE"
    
    # Make sure we're in the right directory (catkin_ws)
    cd ..
    
    rosdep install --from-paths src --ignore-src -r -y
    
    if [ $? -eq 0 ]; then
        print_success "Dependencies installed successfully"
        return 0
    else
        print_error "Failed to install dependencies"
        return 1
    fi
}

# Step 6: Make files executable
make_files_executable() {
    print_info "Making Python scripts and launch files executable..." "$BLUE"
    
    # Make Python scripts executable
    if [ -d "src/quadrotor_control/src/scripts" ]; then
        chmod +x src/quadrotor_control/src/scripts/*
        print_success "Python scripts made executable"
    else
        print_warning "Scripts directory not found, skipping..."
    fi
    
    # Make launch files executable
    if [ -d "src/quadrotor_control/launch" ]; then
        chmod +x src/quadrotor_control/launch/*
        print_success "Launch files made executable"
    else
        print_warning "Launch directory not found, skipping..."
    fi
    
    return 0
}

# Step 7: Build workspace
build_workspace() {
    print_info "Building workspace with catkin_make..." "$BLUE"
    
    catkin_make
    
    if [ $? -eq 0 ]; then
        print_success "Workspace built successfully"
        return 0
    else
        print_error "Failed to build workspace"
        return 1
    fi
}

# Step 8: Source workspace
source_workspace() {
    print_info "Sourcing workspace..." "$BLUE"
    
    if [ -f "devel/setup.bash" ]; then
        source devel/setup.bash
        print_success "Workspace sourced successfully"
        return 0
    else
        print_error "Failed to find devel/setup.bash"
        return 1
    fi
}

# Step 9: Setup Gazebo environment
setup_gazebo_environment() {
    print_info "Setting up Gazebo environment..." "$BLUE"
    
    if [ -f "src/quadrotor_description/scripts/setup_env.sh" ]; then
        source src/quadrotor_description/scripts/setup_env.sh
        print_success "Gazebo environment configured"
        return 0
    else
        print_warning "Gazebo environment setup script not found"
        return 0
    fi
}

# Function to display launch instructions
display_launch_instructions() {
    echo ""
    print_info "LAUNCHING THE SIMULATION" "$CYAN"
    echo -e "${CYAN}============================${NC}"
    echo ""
    print_warning "IMPORTANT: You must source the workspace in each new terminal!"
    echo -e "   ${YELLOW}cd $(pwd) && source devel/setup.bash${NC}"
    echo ""
    print_info "Launch Gazebo simulation:"
    echo -e "   ${GREEN}cd $(pwd) && source devel/setup.bash${NC}"
    echo -e "   ${GREEN}roslaunch quadrotor_control simulation.launch${NC}"
    echo ""
    print_info "Launch with custom platform position:"
    echo -e "   ${GREEN}roslaunch quadrotor_control simulation.launch x:=2.0 y:=3.5 yaw:=1.57${NC}"
    echo ""
    print_info "To make sourcing easier, add this to your ~/.bashrc:"
    echo -e "   ${BLUE}echo 'source $(pwd)/devel/setup.bash' >> ~/.bashrc${NC}"
    echo -e "   ${BLUE}source ~/.bashrc${NC}"
}

### Main Execution Flow

# Step 1: Check ROS Noetic
echo -e "${BLUE}Step 1: Checking ROS Noetic...${NC}"
if ! check_ros_noetic; then
    exit 1
fi
echo ""

# Step 2: Setup workspace structure
echo -e "${BLUE}Step 2: Setting up workspace structure...${NC}"
if ! setup_workspace_structure; then
    exit 1
fi
echo ""

# Step 3: Check rosdep
echo -e "${BLUE}Step 3: Checking rosdep...${NC}"
if ! check_rosdep; then
    exit 1
fi
echo ""

# Step 4: Check and install AprilTag ROS
echo -e "${BLUE}Step 4: Checking and installing AprilTag ROS...${NC}"
if ! check_install_apriltag_ros; then
    print_warning "AprilTag ROS installation failed, but continuing with setup..."
fi
echo ""

# Step 5: Install dependencies
echo -e "${BLUE}Step 5: Installing dependencies...${NC}"
if ! install_dependencies; then
    exit 1
fi
echo ""

# Step 6: Make files executable
echo -e "${BLUE}Step 6: Making files executable...${NC}"
make_files_executable
echo ""

# Step 7: Build workspace
echo -e "${BLUE}Step 7: Building workspace...${NC}"
if ! build_workspace; then
    exit 1
fi
echo ""

# Step 8: Source workspace
echo -e "${BLUE}Step 8: Sourcing workspace...${NC}"
if ! source_workspace; then
    exit 1
fi
echo ""

# Step 9: Setup Gazebo environment
echo -e "${BLUE}Step 9: Setting up Gazebo environment...${NC}"
setup_gazebo_environment
echo ""

print_success "Setup completed successfully!" "$GREEN"

# Display launch instructions
display_launch_instructions

exit 0