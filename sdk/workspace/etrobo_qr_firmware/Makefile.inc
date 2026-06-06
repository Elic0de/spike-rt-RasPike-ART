KERNEL_LIB=
CFLAGS += -DTOPPERS_OMIT_SYSLOG

APPL_COBJS +=\
line_controller.o\
serial_protocol.o\

APPL_LIBS += -lm
