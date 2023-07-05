KERNEL_DIR=/home/ray/workspace/kernel_dev/ebf_4.19.35_imx6ul
MNT_DIR=/home/ray/workdir

ARCH=arm
CROSS_COMPILE=arm-linux-gnueabihf-
export  ARCH  CROSS_COMPILE

obj-m :=spi_st7735s_fb.o

all:app
	$(MAKE) -C $(KERNEL_DIR) M=$(CURDIR) modules

app:	
	arm-linux-gnueabihf-gcc fb_app.c -o fb_app

.PHONE:clean copy 

clean:
	$(MAKE) -C $(KERNEL_DIR) M=$(CURDIR) clean
	sudo rm $(CURDIR)/fb_app

copy:
	sudo cp $(CURDIR)/spi_st7735s_fb.ko $(CURDIR)/fb_app  $(MNT_DIR) 