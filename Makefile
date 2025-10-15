obj-m += tmi8152_spi_dev.o
obj-m += motor.o

all:
	$(MAKE) -C $(KSRC) M=$(PWD)	modules

clean:
	$(MAKE) -C $(KSRC) M=$(PWD)	clean

.PHONY: all clean
