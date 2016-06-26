# TI_LauchPad_USB_LinuxKernelDriver
This is the start code to create more complex USB linux kernel drivers on PC side and develop the HW code on the Cortex-M4 side. The steps which needs to be done which can be used like work instructions could be found in the Wiki page for this repository.

# The GIT work instructions
The following project files should be checked into source control:

    .ccsproject
    .cproject
    .project
    .settings folder
    makefile.defs (if using SYS/BIOS)

.ccsproject has project information specific to CCS.

.cproject and .project are Eclipse CDT project files.

.settings folder is a standard Eclipse folder that has settings that apply for the project.

makefiles.defs has additional make rules included by the generated project makefile for SYS/BIOS projects.

The following files and folders should not be checked into source control:

    \Debug or \Release \<configuration name> folder
    .xdchelp
    \.config folder
    \.launches folder

Configuration folders like \Debug or \Release are generated folders that contain build artifacts like object files and do not need to be checked into source control.

.xdchelp is a generated file that is used to help display error messages in the problems view and does not need to be checked into source control.

.config and .launches are generated folders that do not need to be checked into source control 
