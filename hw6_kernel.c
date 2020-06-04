/*
*@file hw6_ram.c
*@author Ramprakash
*@brief pci driver
*@date 5th May 2020
*/

#include<linux/module.h>
#include<linux/types.h>
#include<linux/kdev_t.h>
#include<linux/cdev.h>
#include<linux/slab.h>
#include<linux/uaccess.h>
#include<linux/pci.h>
#include<linux/fs.h>
#include<linux/kernel.h>

//Macros
#define DEVCNT 1
#define DEVNAME "hw6_kernel"

//LED commands
#define LED0_ON 0xe
#define LED0_OFF 0X4e
#define LED1_ON 0xe00
#define LED1_OFF 0x4e00
#define LED2_ON 0xe0000
#define LED2_OFF 0x4e0000
#define LED3_ON 0xe000000
#define LED3_OFF 0x4E000000
#define LEDS_OFF (LED0_OFF | LED1_OFF | LED2_OFF | LED3_OFF)


//Global vars
//static int mod1=40;
//module_param(mod1,int,S_IRUSR | S_IWUSR);


static int blink_rate = 2;
module_param(blink_rate,int,S_IRUSR | S_IWUSR);
static struct timer_list my_timer;
static int open_close_status=0;
//static int *kbuffer;
static int on_off_state=1;

//static int pci_blinkdriver_probe(struct pci_dev *pdev, size_t len, loff_t *offset);
//static void pci_blinkdriver_remove(struct pci_dev *pdev);
//static void my_callback(struct timer_list *list);

static struct class *my_func;

//struct for char driver
static struct my_chardev_struct{
	struct cdev my_cdev;	//ref for cdev
	dev_t mydev_node;	//device node
	unsigned int syscall_val;
	unsigned long led_start_addr;
}mydev;

//PCI struct
static struct my_pcidev{
	struct pci_dev *pdev;
	void *mem_addr;
}my_pcidev;

//Callback function for timer:
static void my_callback(struct timer_list *list){
	printk("call back entered\n");
	if(open_close_status){
		if(on_off_state){
			writel(LED0_ON,my_pcidev.mem_addr+0xE00);
			on_off_state = 0;
			}
		else{
			writel(LED0_OFF,my_pcidev.mem_addr+0xE00);
			on_off_state = 1;
		}
	}
	mod_timer(&my_timer,jiffies+msecs_to_jiffies(mydev.syscall_val));
	printk("callback exit\n");
}



//Timer Init

void timer_init(void){
	timer_setup(&my_timer, my_callback, 0);
	printk(KERN_INFO"Timer creaeted successfully");
	mydev.syscall_val = 1000/blink_rate;
	printk("1/blinkrate=%d\n",mydev.syscall_val);
	//jiffies are eqal to sec(1/hz)
	mod_timer(&my_timer,jiffies+msecs_to_jiffies(mydev.syscall_val));
	printk(KERN_INFO"mod timer executed successfully\n");
}


//File operations
//open file

static int file_open(struct inode *inode, struct file *file){
	printk(KERN_INFO"Kernel: File Opened \n");
	//timer init
	timer_init();
	//Toggle ON "open/close" status 
	open_close_status = 1;
	return 0;
}

//release file

static int file_release(struct inode *inode, struct file *file){
	printk(KERN_INFO"Kernel: File released\n");
	open_close_status = 0;
	del_timer_sync(&my_timer);
	return 0;
}

//file_read
static ssize_t file_read(struct file *file, char __user *buf, size_t len, loff_t *offset){
	int ret;
	printk(KERN_INFO"before readl, my_pcidev.mem_addr=%p",my_pcidev.mem_addr);

	printk(KERN_INFO"ADDED ADDR: %p",my_pcidev.mem_addr+0xE00);
	mydev.syscall_val= readl(my_pcidev.mem_addr+0xE00);
	printk(KERN_INFO"after readl, mypcidev.mem_addr+0xE00 ret val=%u",mydev.syscall_val);
	if(!buf){
		ret = -EINVAL;
		goto out;
	}

	printk(KERN_INFO"Read memory address %u \n",mydev.syscall_val);
	if(copy_to_user(buf,&mydev.syscall_val,len)){
		printk(KERN_ERR"Copy to user error \n");
		ret = -EFAULT;
		goto out;
	}
	ret = sizeof(int);
out:
	return ret;
}

//file_write
static ssize_t file_write(struct file *file, const char __user *buf, size_t len, loff_t *offset){
	int *kern_buf;
	int ret;
	if(!buf){
		ret=-EINVAL;
		goto out;
	}	
	kern_buf = kmalloc(len,GFP_KERNEL);
	if(!kern_buf){
		ret=-ENOMEM;
		goto out;
	}
	printk("len-%ld\n",len);
	if(copy_from_user(kern_buf,buf,len)){
		ret = -EFAULT;
		goto mem_out;
	}
	ret=len;
	mydev.syscall_val=*kern_buf;
	printk(KERN_INFO"userspace wrote %p to us syscall_val= %u \n",kern_buf,mydev.syscall_val);
	writel(mydev.syscall_val,my_pcidev.mem_addr+0xE00);

	if(mydev.syscall_val > 0){
		blink_rate = mydev.syscall_val;
		mydev.syscall_val = 1/blink_rate;
		mydev.syscall_val = mydev.syscall_val * 1000;
		mod_timer(&my_timer,jiffies+msecs_to_jiffies((unsigned int)mydev.syscall_val));		
		printk(KERN_INFO"timer blink rate updated");
	}
	else if(mydev.syscall_val == 0)
		printk(KERN_INFO"no update to blink rate(0)\n");
	else
		return EINVAL;
	
mem_out:
	kfree(kern_buf);
out:
	return ret;
}


//PCI device functions
//PCI device id
static const struct pci_device_id pci_ids[]={
	{ PCI_DEVICE(0x8086, 0x100e)},
	{}	
};

static int pci_probe(struct pci_dev *pdev,const struct pci_device_id *ent){
	resource_size_t mem_start, mem_len;
	unsigned long barmask;
	
	printk(KERN_INFO"probe function called\n");
	
	//get base address
	barmask = pci_select_bars(pdev,IORESOURCE_MEM);
	printk(KERN_INFO"barmask=%lx",barmask);
	//get the pci selected regions
	if(pci_request_selected_regions(pdev,barmask,"pcidriver")){
		printk(KERN_ERR"selected req region failed\n");
		goto unregister_selected_regions;
	}
	mem_start=pci_resource_start(pdev,0);
	mem_len = pci_resource_len(pdev,0);
	printk(KERN_INFO"mem start:%lx\n",(unsigned long)mem_start);
	printk(KERN_INFO"mem len:%lx\n",(unsigned long)mem_len);
	
	if(!(my_pcidev.mem_addr=ioremap(mem_start,mem_len))){
		printk(KERN_ERR"ioremap failed\n");
		goto unregister_ioremap;
	}
	//if device is working
	mydev.led_start_addr=readl(my_pcidev.mem_addr);
	printk(KERN_INFO"Initial val is=%lx\n",mydev.led_start_addr);
	return 0;
unregister_ioremap:
	iounmap(my_pcidev.mem_addr);
unregister_selected_regions:
	pci_release_selected_regions(pdev,pci_select_bars(pdev,IORESOURCE_MEM));
	return -1;	
}

//pci_remove
static void pci_remove(struct pci_dev *pdev){
	printk(KERN_INFO"removing PCI driver \n");
	iounmap(my_pcidev.mem_addr);
	pci_release_selected_regions(pdev,pci_select_bars(pdev,IORESOURCE_MEM));

}



//PCI file ops
static struct file_operations mydev_fops={
	.owner = THIS_MODULE,
	.open = file_open,
	.read = file_read,
	.write = file_write,
	.release = file_release,

};


//PCI driver
static struct pci_driver my_pcidriver={
	.name = "hw4_pcidriver",
	.id_table = pci_ids,
	.probe = pci_probe,
	.remove = pci_remove,
};

//Module Initialization
static int __init myfunc_init(void){
	int ret_pci;
	mydev.syscall_val = 40;
	//Check for param value
	//if(mydev.syscall_val != mod1){
	//	mydev.syscall_val=mod1;
	//}
	printk(KERN_INFO"kernel initialization\n");

	//Initializing char drivers
	if(alloc_chrdev_region(&mydev.mydev_node,0,DEVCNT,DEVNAME)){
		printk(KERN_ERR"Dyamic Memory allocation failed\n");
		return -1;
	}

	if((my_func = class_create(THIS_MODULE,DEVNAME))==NULL){
		printk(KERN_ERR"class create failed\n");
		class_destroy(my_func);
	}
	printk(KERN_INFO"return my_func=%ln",(unsigned long*)my_func);

	//if char driver is successfully allocated
	printk(KERN_INFO"Allocated %d devices at major %d\n",DEVCNT, MAJOR(mydev.mydev_node));
	cdev_init(&mydev.my_cdev,&mydev_fops);
	mydev.my_cdev.owner=THIS_MODULE;
	//Register PCI driver
	ret_pci=pci_register_driver(&my_pcidriver);
	if(ret_pci<0){
		pci_unregister_driver(&my_pcidriver);
		printk(KERN_ERR"PCI driver registration failed\n");
		return -1;
	}
	//PCI driver successfully registered
	printk(KERN_INFO"pci device reg success\n");
	if(cdev_add(&mydev.my_cdev,mydev.mydev_node,DEVCNT)){
		printk(KERN_ERR"cdev_add failed\n");
		unregister_chrdev_region(mydev.mydev_node,DEVCNT);
		return -1;
	}
	printk(KERN_INFO"Cdev reg success\n");
	//Creating device node automatically
	//class_device_create(my_func,NULL,mydev.devnode,NULL,"autodrv");

	if(device_create(my_func,NULL,mydev.mydev_node,NULL,DEVNAME)==NULL){
		printk(KERN_ERR"device create failure\n");
		class_destroy(my_func);
	}
	return 0;
}
void __exit myfunc_exit(void){
	pci_unregister_driver(&my_pcidriver);
	cdev_del(&mydev.my_cdev);
	unregister_chrdev_region(mydev.mydev_node,DEVCNT);
	printk(KERN_INFO"module unloaded\n");
	
}



MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ram");
MODULE_VERSION("0.1");
module_init(myfunc_init);
module_exit(myfunc_exit);
