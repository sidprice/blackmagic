ifneq ($(V), 1)
MFLAGS += --no-print-dir
Q := @
endif

CONFIG_BMDA =
NO_LIBOPENCM3 =
ifeq ($(PROBE_HOST), hosted)
	CONFIG_BMDA = true
	NO_LIBOPENCM3 = true
endif

all:
ifndef NO_LIBOPENCM3
	$(Q)if [ ! -f deps/libopencm3/Makefile ]; then \
		echo "Initialising git submodules..." ;\
		git submodule update --init ;\
	fi
	$(Q)$(MAKE) $(MFLAGS) -C deps/libopencm3 lib/stm32/f1 lib/stm32/f4 lib/stm32/f7 lib/lm4f
endif
	$(Q)$(MAKE) $(MFLAGS) -C src

all_platforms:
	$(Q)$(MAKE) $(MFLAGS) -C src $@

clean:
ifndef NO_LIBOPENCM3
	$(Q)$(MAKE) $(MFLAGS) -C deps/libopencm3 $@
endif
	$(Q)$(MAKE) $(MFLAGS) -C src $@

clang-tidy: SYSTEM_INCLUDE_PATHS=$(shell pkg-config --silence-errors --cflags libusb-1.0 libftdi1)
clang-tidy:
	$(Q)scripts/run-clang-tidy.py -s "$(PWD)" $(SYSTEM_INCLUDE_PATHS)

clang-format:
	$(Q)$(MAKE) $(MFLAGS) -C src $@

.PHONY: clean all_platforms clang-tidy clang-format
