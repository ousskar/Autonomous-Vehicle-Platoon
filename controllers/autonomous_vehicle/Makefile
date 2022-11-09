# Webots Makefile system 
#
# You may add some variable definitions hereafter to customize the build process
# See documentation in $(WEBOTS_HOME_PATH)/resources/Makefile.include

ifndef WEBOTS_AUTOMOBILE_PATH
WEBOTS_AUTOMOBILE_PATH = $(WEBOTS_MODULES_PATH)/projects/automobile/libraries
endif

INCLUDE = -I"$(WEBOTS_AUTOMOBILE_PATH)/driver/include"
LIBRARIES = -L"$(WEBOTS_AUTOMOBILE_PATH)/driver" -ldriver -L"$(WEBOTS_AUTOMOBILE_PATH)/car" -lcar

# Do not modify the following: this includes Webots global Makefile.include
space :=
space +=
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
