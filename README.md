Im using a libopencm3 template for this midi project
And I'm also using ubuntu as an OS and STM32f103c8 (Blue Pill) for the board
Here is the link if you want to check it out
https://github.com/libopencm3/libopencm3-template.git

# Preparation
 1. sudo apt install stlink-tools gcc-arm-none-eabi # (if all the tools not installed yet)
 2. git clone --recurse-submodules https://github.com/arbipink/midi-project-with-libopencm3.git
 3. cd midi-project
 4. make -C libopencm3 # (Only needed once)
 5. make -C my-project

# How to run the code
 1. cd my-project
 2. st-flash --reset write midi-project.bin 0x8000000
