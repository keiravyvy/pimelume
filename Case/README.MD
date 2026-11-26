You doing a project of converting a broken mouse into a bluetooth wireless mouse with gesture controls. You are using this mouse:

Mouse: Redragon Sniper Pro M801P-RGB

Weight & Size

Product Weight: 0.159kg
Product Size(L x W x H): 12.2*8.2*4.2cm

Components:
- Raspberry Pi Pico W
- 2xAA Batteries
- IMU module

All the guts of the mouse have been taken out. The idea is to reuse the main PCB of the mouse and and use the Pico W to sniff the mouse signals and relay them to the PC over bluetooth, then also use the IMU to add gesture scrolling where the user picks up the mouse and tilts to scroll.

Your task is to design the top part of the case for the mouse, we will be resuing the bottom part as it has the PCB already mounted and the IR LED and camera already calibrated. So the case should fit on top of the bottom part (the base should be identical in shape) the top case will be 3D printed. Ideally it should be only a few parts. The case should actuate the left/right mouse buttons which are already on the PCB. The scroll should also be reused if possible, but it is not required. The bottom side of the case (Where the battery used to be) is emptied, so there should be enough space for a standard 2xAA battery holder (AA not AAA). The case needs to have enough space there for the battery holder, a Pi Pico W with a small BNO080 9-Axis IMU Module. There should also be enough space to attach a cable to the type C port and route it back inside the mouse so keep like a 1cmx1cm hole for that. the mouse doesn't have to look smooth and professional. It should feel okay to hold and there should be holes for the battery holder, and the cable.

There are reference images of the mouse under MouseCase/ReferenceImages whch I took of the mouse with a ruler.

Use build123d to design and review the case. Export out a FreeCAD file at the end where I can make minor modifications if needed, also export out a .stl file ready for printed. It's already installed in MouseCase/.venv. Write the python scripts in the MouseCase/ folder.
