#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/platform_device.h> 
#include <linux/delay.h>
#include <mach/platform.h>
#if 0
enum kobject_action {
       KOBJ_ADD,
       KOBJ_REMOVE,
       KOBJ_CHANGE,
       KOBJ_MOVE,
       KOBJ_ONLINE,
       KOBJ_OFFLINE,
       KOBJ_MAX
};
#endif

//debug
//#define FIIO_DEBUG_TP
#ifdef FIIO_DEBUG_TP
#define fiio_debug(x...)  printk(KERN_INFO "[fiio_uevent] " x)
#else
#define fiio_debug(x...)
#endif
static struct class *remotectl_event_class;
static struct device *fiio_remotectl_event_device;
static struct kobject *fiio_remotectl_event_object;

static int fiio_player_mode = 0;

static ssize_t player_mode_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	char *mode = "BT";
	if (0 == fiio_player_mode) {
		mode="LOCAL";
	}
	else {
		mode="BT";
	}
out:
	return sprintf(buf, "%s\n", mode);
}

static DEVICE_ATTR(player_mode, S_IRUGO, player_mode_show, NULL);
static struct device_attribute *fiio_state_attributes[] = {
	&dev_attr_player_mode,
	NULL
};

static int __init remotectl_event_init(void)
{
	struct device_attribute **attrs = fiio_state_attributes;
	struct device_attribute *attr;
	int err;
	remotectl_event_class = class_create(THIS_MODULE,"fiio");
	err = PTR_ERR(remotectl_event_class);
	if (IS_ERR(remotectl_event_class)) {
		printk("create fiio class !\n");
		goto out;
	}
	fiio_remotectl_event_device = device_create(remotectl_event_class,NULL,MKDEV(0,0),NULL,"uevent");
	err = PTR_ERR(fiio_remotectl_event_device);
	if (IS_ERR(fiio_remotectl_event_device)) {
		printk("create fiio uevent !\n");
		goto outclass;
	}
	fiio_remotectl_event_object = &fiio_remotectl_event_device->kobj;
	//add attr
	while ((attr = *attrs++)) {
		err = device_create_file(fiio_remotectl_event_device, attr);
		if (err) {
			device_destroy(remotectl_event_class, fiio_remotectl_event_device->devt);
			return err;
		}
	}
	return 0;
outclass:
	class_destroy(remotectl_event_class);
out:
	return err;
}
/*
 *remove
 */
static void __exit remotectl_event_exit(void) {
	device_destroy(remotectl_event_class,MKDEV(0,0));
	class_destroy(remotectl_event_class);
}

/*
* name:fiio_remotectl_event_send
* function:send uevent
* params
* uevent_type:
* sprintf(spectator, "DEVPATH=%s", "/sys/devices/virtual/fiio/uevent");
* sprintf(subcode, "%s=%d", uevent_name,uevent);
**/
void fiio_remotectl_event_send(char* uevent_name,int uevent_type) 
{
	fiio_debug("%s fiio send uevent type %s=%d\n",__func__,uevent_name,uevent_type);
	char spectator[50];
	char subcode[50];
	char *envp[] = {spectator,subcode, NULL };
	sprintf(spectator, "DEVPATH=%s", "/sys/devices/virtual/fiio/uevent");
	sprintf(subcode, "%s=%d", uevent_name,uevent_type);
	kobject_uevent_env(fiio_remotectl_event_object, KOBJ_CHANGE, envp);
}
EXPORT_SYMBOL(fiio_remotectl_event_send);


void fiio_remotectl_event_send_player(char* uevent_name,int uevent_type) 
{
	fiio_debug("%s fiio send uevent type %s=%d\n",__func__,uevent_name,uevent_type);
	char spectator[50];
	char subcode[50];
	char *envp[] = {spectator,subcode, NULL };
	sprintf(spectator, "DEVPATH=%s", "/sys/devices/virtual/fiio/uevent");
	sprintf(subcode, "%s=%d", uevent_name,uevent_type);
	kobject_uevent_env(fiio_remotectl_event_object, KOBJ_CHANGE, envp);
	fiio_player_mode = uevent_type;
}
EXPORT_SYMBOL(fiio_remotectl_event_send_player);

module_init(remotectl_event_init);
module_exit(remotectl_event_exit);

MODULE_AUTHOR("pengweizhong <pengweizhong@fiio.net>");
MODULE_DESCRIPTION("driver for fiio uevent");
MODULE_LICENSE("GPL");
