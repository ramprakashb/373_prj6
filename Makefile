KERNEL_DIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

obj-m   :=hw6_kernel_trial.o
//obj-m	:= hw6_kernel.o
//obj-m := hw4_ram_kernel.o
//obj-m += ledmon.o

default:
	$(MAKE) -C $(KERNEL_DIR) SUBDIRS=$(PWD) modules

clean:
	$(MAKE) -C $(KERNEL_DIR) SUBDIRS=$(PWD) clean

