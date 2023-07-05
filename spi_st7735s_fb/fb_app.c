/*
 * @author: PeiRui Wang
 * @description: st7735s framebuffer driver test APP, test on Fire IMX6ULL board
 *  @kernel version: Linux_4.19.35
 * @year :  2023
 */
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <string.h>
#include <linux/fb.h>
#include <unistd.h>
 
#define RED  		0xFF0000
#define GREEN	0x07E000

static int fd_fb;
static struct fb_var_screeninfo var;	/* Current var */
static int screen_size;
static unsigned char *fb_base;
static unsigned int line_width;
static unsigned int pixel_width;
 
void lcd_put_pixel(int x, int y, unsigned int color)
//传入的 color 表示颜色，它的格式永远是 0x00RRGGBB，即 RGB888。
//当 LCD 是 16bpp 时，要把 color 变量中的 R、 G、 B 抽出来再合并成 RGB565 格式。
{
    unsigned char *pen_8 = fb_base+y*line_width+x*pixel_width;
    //计算(x,y)坐标上像素对应的 Framebuffer 地址。
 
    unsigned short *pen_16;
    unsigned int *pen_32;
    
    unsigned int red, green, blue;
    
    pen_16 = (unsigned short *)pen_8;
    pen_32 = (unsigned int *)pen_8;
    
    switch (var.bits_per_pixel)
    {
      //对于 8bpp， color 就不再表示 RBG 三原色了，这涉及调色板的概念， color 是调色板的值。
        case 8:
            {
                *pen_8 = color;
                break;
            }
        case 16:
            {
                 /* 565 */
                //先从 color 变量中把 R、 G、 B 抽出来。
                red = (color >> 16) & 0xff;
                green = (color >> 8) & 0xff;
                blue = (color >> 0) & 0xff;
                //把 red、 green、 blue 这三种 8 位颜色值，根据 RGB565 的格式，
                //只保留 red 中的高 5 位、 green 中的高 6 位、 blue 中的高 5 位，
                //组合成一个新的 16 位颜色值。
                color = ((red >> 3) << 11) | ((green >> 2) << 5) | (blue >> 3);
                //把新的 16 位颜色值写入 Framebuffer
                *pen_16 = color;
                break;
            }
        case 32:
            {
                //对于 32bpp，颜色格式跟 color 参数一致，可以直接写入Framebuffer
                *pen_32 = color;
                break;
            }
        default:
            {
                printf("can't surport %dbpp\n",var.bits_per_pixel);
                break;
            }
     }
}

void draw_cube(int bias, unsigned int color)
{
    memset(fb_base, 0xff, screen_size);
    int i;
    int len = 40;
	for (i = 0; i < len; i++)
    {
        lcd_put_pixel(var.xres/2+i-bias, var.yres/2-bias, color);
        lcd_put_pixel(var.xres/2-bias, var.yres/2+i-bias, color);

        lcd_put_pixel((var.xres/2+19-bias)+i,( var.yres/2+19-bias), color);
        lcd_put_pixel((var.xres/2+19-bias),( var.yres/2+19-bias)+i, color);

        lcd_put_pixel((var.xres/2+19-bias)+i,( var.yres/2+19-bias+len), color);
        lcd_put_pixel((var.xres/2+19-bias+len),( var.yres/2+19-bias)+i, color);

        //usleep(10000);
    }
    for (i = 0; i < 20; i++)
    {
        lcd_put_pixel(var.xres/2+i-bias, var.yres/2+i-bias, color);
        lcd_put_pixel(var.xres/2+i-bias, var.yres/2+i-bias+len, color);
        lcd_put_pixel(var.xres/2+i-bias+len, var.yres/2+i-bias, color);
        //usleep(10000);
    }
}
int main(int argc,int **argv)
{	
	int i,j;
	fd_fb = open("/dev/fb2", O_RDWR);
	if (fd_fb < 0)
	{
		printf("can't open fb file!\n");
		return -1;
	}	
	if (ioctl(fd_fb, FBIOGET_VSCREENINFO, &var))
	{
		printf("can't get var\n");
		return -1;
	}
	
	line_width = var.xres * var.bits_per_pixel / 8;
	pixel_width = var.bits_per_pixel / 8;
	screen_size = var.xres * var.yres * var.bits_per_pixel / 8;
    printf("var.bits_per_pixel:%d\n", var.bits_per_pixel);
    printf("var.xres:%d\n", var.xres);
    printf("var.yres:%d\n", var.yres);
	
	fb_base = (unsigned char *)mmap(NULL , screen_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd_fb, 0);
	if (fb_base == (unsigned char *)-1)
	{
		printf("can't mmap\n");
		return -1;
	}
	/* 清屏: 全部设为白色 */
	memset(fb_base, 0xf0, screen_size);
 
	/* 绘制图形 */
    int delay = 30000;
    for(j=0;j<4;j++)
    {
        for(i=30;i<=60;i++)
        {
            draw_cube(i,RED);
            usleep(delay);
        }
        for(i=60;i>=0;i--)
        {
            draw_cube(i,GREEN);
            usleep(delay);
        }
        for(i=0;i<=30;i++)
        {
            draw_cube(i,RED);
            usleep(delay);
        }
        delay-=5000;
    }

    munmap(fb_base, screen_size);
	close(fd_fb);
	return 0;
}