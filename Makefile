-include .localenv

ifeq ($(RTE_SDK),)
$(error "Please define RTE_SDK environment variable")
endif

# Default target, can be overriden by command line or environment
ifeq ($(RTE_TARGET),)
$(error "Please define RTE_TARGET environment variable")
endif

include $(RTE_SDK)/mk/rte.vars.mk

APP = nwtest

SRCS-y := nwtest.c

include $(RTE_SDK)/mk/rte.extapp.mk
