#----------------------------------------------------------
# Include the utility files
#----------------------------------------------------------

UTILITIES_ROOT = $(SDK_ROOT)/platform/utilities
SOURCES += $(UTILITIES_ROOT)/src/sw_timer.c \
           $(UTILITIES_ROOT)/src/fsl_debug_uart.c \
           $(UTILITIES_ROOT)/src/fsl_os_abstraction_bm.c
