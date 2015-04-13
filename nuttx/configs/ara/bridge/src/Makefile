-include $(TOPDIR)/Make.defs

CFLAGS		+= -I$(TOPDIR)/sched

ASRCS		=
AOBJS		= $(ASRCS:.S=$(OBJEXT))

CSRCS		= up_boot.c

ifeq ($(CONFIG_PWM),y)
CSRCS		+= up_pwm.c
endif

ifeq ($(CONFIG_BOARD_HAVE_DISPLAY), y)
ifeq ($(CONFIG_APB1),y)
CSRCS   += dsi_rx_init.c
else
CSRCS   += lg4892.c dsi_tx_init.c
endif
endif

COBJS		= $(CSRCS:.c=$(OBJEXT))

SRCS		= $(ASRCS) $(CSRCS)
OBJS		= $(AOBJS) $(COBJS)


GIT_VERSION := $(shell git describe --dirty --always 2>/dev/null)
BUILD_TIME := $(shell date)
BUILD_USER := $(shell whoami)
BUILD_HOST := $(shell hostname)

BUILD_INFO := $(shell echo "($(BUILD_USER)@$(BUILD_HOST)) $(BUILD_TIME)")

ARCH_SRCDIR	= $(TOPDIR)/arch/$(CONFIG_ARCH)/src
ifeq ($(WINTOOL),y)
	CFLAGS	+= -I "${shell cygpath -w $(ARCH_SRCDIR)/chip}" \
		   -I "${shell cygpath -w $(ARCH_SRCDIR)/common}" \
		   -I "${shell cygpath -w $(ARCH_SRCDIR)/armv7-m}"
else
	CFLAGS	+= -I$(ARCH_SRCDIR)/chip -I$(ARCH_SRCDIR)/common -I$(ARCH_SRCDIR)/armv7-m
ifneq ($(GIT_VERSION), )
	CFLAGS  += -DARA_FW_VERSION=\""$(GIT_VERSION)"\"
endif
ifneq ($(BUILD_INFO), )
	CFLAGS  += -DARA_FW_BUILD_TIME=\""$(BUILD_INFO)"\"
endif
endif

all: libboard$(LIBEXT)

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS) $(LINKOBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

libboard$(LIBEXT): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

.depend: Makefile $(SRCS)
	$(Q) $(MKDEP) $(CC) -- $(CFLAGS) -- $(SRCS) >Make.dep
	$(Q) touch $@

depend: .depend

clean:
	$(call DELFILE, libboard$(LIBEXT))
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
