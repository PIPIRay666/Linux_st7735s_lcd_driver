KERNEL_DIR=/home/ray/workspace/kernel_dev/ebf_4.19.35_imx6ul
MNT_DIR=/home/ray/workdir

ARCH=arm
CROSS_COMPILE=arm-linux-gnueabihf-
export  ARCH  CROSS_COMPILE

obj-m :=spi_st7735s_org.o

all:#app
	$(MAKE) -C $(KERNEL_DIR) M=$(CURDIR) modules

#app:	
#arm-linux-gnueabihf-gcc lcd_app.c -o lcd_app

.PHONE:clean copy 

clean:
	$(MAKE) -C $(KERNEL_DIR) M=$(CURDIR) clean
#sudo rm $(CURDIR)/lcd_app

copy:
#sudo cp $(CURDIR)/spi_st7735s_org.ko $(CURDIR)/lcd_app  $(MNT_DIR) 
	sudo cp $(CURDIR)/spi_st7735s_org.ko $(MNT_DIR) 