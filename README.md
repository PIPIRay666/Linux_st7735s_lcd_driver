# Linux_st7735s_lcd_driver
    IMX6ULL嵌入式Linux平台的系统st7735s显示屏芯片驱动程序，可作为使用SPI总线的LCD/OLED显示驱动移植参考。

## SPI接线说明
    *SPI功能引脚*
    MX6UL_PAD_CSI_DATA05__ECSPI1_SS0	       GPIO4_IO26 --- CS 片选，低电平有效
    MX6UL_PAD_CSI_DATA04__ECSPI1_SCLK 		GPIO4_IO25	--- SCK 时钟
    MX6UL_PAD_CSI_DATA06__ECSPI1_MOSI 	GPIO4_IO27  --- SDA 主发从收
    MX6UL_PAD_CSI_DATA07__ECSPI1_MISO		GPIO4_IO28	主收从发	

    *GPIO功能引脚*
    MX6UL_PAD_CSI_DATA02__GPIO4_IO23	DC 屏幕数据/命令选择
    MX6UL_PAD_CSI_DATA03__GPIO4_IO24	RES 屏幕复位
    MX6UL_PAD_CSI_DATA01__GPIO4_IO22 	BL 背光，低电平关闭

## !DTS设备树修改参考!
```
/*IOMUXC节点添加相关pin*/
	pinctrl_st7735s: st7735s {  
		fsl,pins = <
			MX6UL_PAD_CSI_DATA03__GPIO4_IO24        0x10B0	//屏幕复位RES引脚
			MX6UL_PAD_CSI_DATA02__GPIO4_IO23        0x10B0  //屏幕DC(data or command)引脚
			MX6UL_PAD_CSI_DATA01__GPIO4_IO22        0x10B0  //屏幕BL背光引脚
		>;
	};  

	pinctrl_ecspi1:ecspi1grp {
		fsl,pins = <
			MX6UL_PAD_CSI_DATA05__ECSPI1_SS0              0x1a090
			MX6UL_PAD_CSI_DATA04__ECSPI1_SCLK           0x11090
			MX6UL_PAD_CSI_DATA06__ECSPI1_MOSI           0x11090
			MX6UL_PAD_CSI_DATA07__ECSPI1_MISO           0x11090
		>;
 	};

/*添加spi节点*/
&ecspi1 {
	fsl,spi-num-chipselects = <1>;
	cs-gpio = <&gpio4 26 GPIO_ACTIVE_LOW>;	/* 设置片选引脚，供子节点使用 */ 
	pinctrl-names = "default";
	pinctrl-0 = <&pinctrl_ecspi1>;
 	status = "okay";
	#address-cells = <1>;
	#size-cells = <0>; 
    
	lcd: st7735s@0 {
		compatible = "fire,st7735s";
		reg = <0>;						/* reg = <index>;  指定片选引脚，就是父节点的cs-gpio中第index个 */
		spi-max-frequency = <8000000>;	  /* 指定设备的最高速度 */
		/* GPIO_ACTIVE_HIGH目的是指定有效电平, 使用gpiod_set_value()设置的是逻辑电平 */
		dc-gpio = <&gpio4 23 GPIO_ACTIVE_HIGH>;		/* 数据/命令配置引脚 */
		res-gpio = <&gpio4 24 GPIO_ACTIVE_HIGH>;	/* 复位引脚 */
		bl-gpio = <&gpio4 22 GPIO_ACTIVE_HIGH>;	/* 背光引脚 */
	};
```    
设备树修改完成后记得编译并替换！

## spi_st7735s_org文件夹
    不使用framebuffer的驱动，直接加载编译好的ko模块即可看到效果。
    运行效果：
 ![img](https://github.com/PIPIRay666/Linux_st7735s_lcd_driver/blob/main/pics/org.gif)
## spi_st7735s_fb文件夹
    基于framebuffer的驱动，fb_app.c为framebuffer测试APP源码，请加载编译好的ko模块后并运行fb_app。
    运行效果：
![img](https://github.com/PIPIRay666/Linux_st7735s_lcd_driver/blob/main/pics/fb.gif)

Makefile均位于文件夹内，按照移植的实际情况更改。

## 关于作者

```c
#define nickName "Ray";
```
