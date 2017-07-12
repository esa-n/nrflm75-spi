TARGET = nrflm75-spi
OBJECT = rf24-spi

ARCH = stm8
LIBDIR = ~/stm8/library
SDCC = sdcc
CFLAGS = -c -l$(ARCH) -m$(ARCH)
LFLAGS = -l$(ARCH) -m$(ARCH) --out-fmt-ihx
OBJGROUP = $(TARGET).rel $(OBJECT:%=%.rel)

all: $(OBJGROUP)
	$(SDCC) $(LFLAGS) $(OBJGROUP)

clean:
	rm -f *.ihx *.rel *.map *.rst *.sym *.cdb *.lk *.asm *.lst

%.rel: %.c
	$(SDCC) $(CFLAGS) $<
%.rel: $(LIBDIR)/%.c
	$(SDCC) $(CFLAGS) $<
