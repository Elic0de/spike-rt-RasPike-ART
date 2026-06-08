VERSION=$(shell cat ../../sample/raspike/spike_version)

MAJOR := $(word 1,$(subst ., ,$(VERSION)))
MINOR := $(word 2,$(subst ., ,$(VERSION)))
PATCH := $(word 3,$(subst ., ,$(VERSION)))

RASPIKE_MEASURE_USB_LOOP ?= 1

CFLAGS+=-DTOPPERS_OMIT_SYSLOG -DSPIKE_EXPECTED_VERSION_MAJOR=$(MAJOR)  -DSPIKE_EXPECTED_VERSION_MINOR=$(MINOR) -DSPIKE_EXPECTED_VERSION_PATCH=$(PATCH)

ifeq ($(RASPIKE_MEASURE_USB_LOOP),1)
CFLAGS+=-DRASPIKE_MEASURE_USB_LOOP=1
endif

$(OBJDIR)/raspike.o:../../sample/raspike/spike_version
