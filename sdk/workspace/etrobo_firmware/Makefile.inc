KERNEL_LIB=
CFLAGS += -DTOPPERS_OMIT_SYSLOG

mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

SRCLANG = c++

APPL_CXXOBJS +=\
FirmwareApplication.o\
LineController.o\
MissionManager.o\
SerialProtocol.o\

APPL_DIRS +=\
$(mkfile_path)src/app\
$(mkfile_path)src/comm\
$(mkfile_path)src/lib/control\
$(mkfile_path)src/mission\

INCLUDES +=\
-I$(mkfile_path)include\
-I$(mkfile_path)include/app\
-I$(mkfile_path)include/comm\
-I$(mkfile_path)include/lib/control\
-I$(mkfile_path)include/mission\

APPL_LIBS += -lm
