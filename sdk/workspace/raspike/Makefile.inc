VERSION=$(shell cat $(APPLDIRS)/spike_version)

MAJOR := $(word 1,$(subst ., ,$(VERSION)))
MINOR := $(word 2,$(subst ., ,$(VERSION)))
PATCH := $(word 3,$(subst ., ,$(VERSION)))

# Name of source files is raspike.*, not app.*
APPLNAME=raspike

# Rebuild kernel with non-standard options (eg. TOPPERS_OMIT_SYSLOG)
KERNEL_LIB=

CFLAGS+=-DTOPPERS_OMIT_SYSLOG -DSPIKE_EXPECTED_VERSION_MAJOR=$(MAJOR)  -DSPIKE_EXPECTED_VERSION_MINOR=$(MINOR) -DSPIKE_EXPECTED_VERSION_PATCH=$(PATCH)

$(BUILDDIR)/dep/raspike.o: $(APPLDIRS)/spike_version

mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))
project_shared_path := $(mkfile_path)_project_shared

APPL_COBJS += hub_runtime_state.o hub_fault_injection.o line_trace_controller.o line_trace_speed_controller.o course_tracking_controller.o
APPL_DIRS += $(project_shared_path)/hub_runtime $(project_shared_path)/line_trace_control $(project_shared_path)/control
INCLUDES += -I$(project_shared_path)/hub_runtime -I$(project_shared_path)/line_trace_control -I$(project_shared_path)/control
