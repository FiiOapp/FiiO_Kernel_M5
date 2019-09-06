#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/fs.h>


static int key_vol_open (struct inode * inode, struct file * file);
static ssize_t key_vol_read(struct file * file, char __user * userbuf, size_t count, loff_t * off);

static const struct file_operations key_vol_ops = {
	.open = key_vol_open,
	.read = key_vol_read,
	.owner = THIS_MODULE,
};

static int key_vol_major;
static struct class *key_vol_class;
struct device *key_vol_dev;
static unsigned long *gpb_inl = NULL;


static int key_vol_open (struct inode * inode, struct file * file){
          printk("second_drv_open\n");
          return 0;
}
static ssize_t key_vol_read(struct file * file, char __user * userbuf, size_t count, loff_t * off){
        unsigned int key_vol_value = 0;
            int ret;
        if(count != sizeof(key_vol_value)){
            printk("the count must be sizeof(unsigned int)\n");
            return -1;
        }
        key_vol_value = *gpb_inl;
        printk("##############key_vol_value : %x\n",key_vol_value);
        printk("##############key_vol_value &(1<< 19: %d\n",(key_vol_value & (1<< 19)));
        ret = copy_to_user(userbuf, &key_vol_value, sizeof(key_vol_value));
         if(ret){
            printk("copy error\n");
            return -1;
        }
        return sizeof(key_vol_value);
}
static int X1000_KeyLevel_Init(void)
{
        key_vol_major = register_chrdev(0,"key_vol", &key_vol_ops);

       if(key_vol_major < 0)
            printk("failes to register key_vol chrdev\n");

        key_vol_class = class_create(THIS_MODULE, "key_vol");

        if(key_vol_class < 0)
            printk("failes to create key_vol class\n");

        key_vol_dev = device_create(key_vol_class, NULL,MKDEV(key_vol_major,0), NULL, "key_vol");
         if(key_vol_dev < 0)
            printk("failes to create key_vol device\n");

         gpb_inl = ioremap(0x10010000, 4);
         if(gpb_inl == NULL){
            printk("failes to ioremap key_vol gpb_inl\n");
         }
        return 0;
}

static void X1000_KeyLevel_Exit(void)
{
        iounmap(gpb_inl);
        device_destroy(key_vol_class, MKDEV(key_vol_major,0));
        class_destroy(key_vol_class);
        unregister_chrdev(key_vol_major,"key_vol");
}
module_init(X1000_KeyLevel_Init);
module_exit(X1000_KeyLevel_Exit);
MODULE_LICENSE("GPL");

