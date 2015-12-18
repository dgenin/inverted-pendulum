
ARMGNU ?= arm-linux-gnueabi

LDOPS = -L /usr/lib/gcc-cross/arm-linux-gnueabi/4.7/ -lgcc
COPS = -Wall -O2 -nostartfiles -ffreestanding -g --build-id=none
#COPS = -Wall -O0 -nostdlib -nostartfiles -ffreestanding -g

gcc : driver.hex driver.bin

all : gcc clang

clean :
	rm -f *.o
	rm -f *.bin
	rm -f *.hex
	rm -f *.elf
	rm -f *.list
	rm -f *.img
	rm -f *.bc
	rm -f *.clang.opt.s

vectors.o : vectors.s
	$(ARMGNU)-as vectors.s -o vectors.o

motor.o : motor.c driver.h
	$(ARMGNU)-gcc $(COPS) -c motor.c -o motor.o

driver.o : driver.c driver.h uart.h
	$(ARMGNU)-gcc $(COPS) -c driver.c -o driver.o

driver.elf : memmap vectors.o driver.o motor.o
	$(ARMGNU)-ld vectors.o motor.o driver.o -g -o driver.elf $(LDOPS) -T memmap
	$(ARMGNU)-objdump -D driver.elf > driver.list

driver.bin : driver.elf
	$(ARMGNU)-objcopy driver.elf -O binary driver.bin

driver.hex : driver.elf
	$(ARMGNU)-objcopy driver.elf -O ihex driver.hex

test.o  : test.c
	$(ARMGNU)-gcc $(COPS) -c $< -o $@

LOPS = -Wall -m32 -emit-llvm
LLCOPS = -march=arm -mcpu=arm1176jzf-s
LLCOPS0 = -march=arm 
LLCOPS1 = -march=arm -mcpu=arm1176jzf-s
COPS = -Wall  -O2 -nostdlib -nostartfiles -ffreestanding -g
OOPS = -std-compile-opts

clang : driver.clang.hex driver.clang.bin


driver.clang.bc : driver.c
	clang $(LOPS) -c driver.c -o driver.clang.bc

driver.clang.opt.elf : memmap vectors.o driver.clang.bc
	opt $(OOPS) driver.clang.bc -o driver.clang.opt.bc
	llc $(LLCOPS) driver.clang.opt.bc -o driver.clang.opt.s
	$(ARMGNU)-as driver.clang.opt.s -o driver.clang.opt.o
	$(ARMGNU)-ld -o driver.clang.opt.elf -T memmap vectors.o driver.clang.opt.o
	$(ARMGNU)-objdump -D driver.clang.opt.elf > driver.clang.opt.list

driver.clang.hex : driver.clang.opt.elf
	$(ARMGNU)-objcopy driver.clang.opt.elf driver.clang.hex -O ihex

driver.clang.bin : driver.clang.opt.elf
	$(ARMGNU)-objcopy driver.clang.opt.elf driver.clang.bin -O binary





