/*
*@file hw4_ram.c
*@author Ramprakash
*@brief pci driver
*@date 5th May 2020
*/


#include<linux/init.h>
#include<linux/module.h>
#include<linux/types.h>
#include<linux/kdev_t.h>
#include<linux/cdev.h>
#include<linux/slab.h>
#include<linux/uaccess.h>
#include<linux/pci.h>
#include<linux/fs.h>
#include<linux/kernel.h>
#include<linux/errno.h>
#include<linux/cdev.h>
#include<linux/moduleparam.h>
#include<linux/kernel.h>
#include<linux/workqueue.h>
#include<linux/timer.h>
#include<linux/errno.h>
#include<linux/usb.h>
#include<linux/slab.h>
#include<linux/delay.h>
#include<linux/interrupt.h>

//Macros
#define DEVCNT 1
#define DEVNAME "ece_led"

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

//Receive deesc address
#define RDBAL 0x2800 //Receive desc base addr low
#define RDBAH 0x2804 //Receive desc base addr high
#define RDLEN 0x2808 //Recevie desc base addr len
#define RDH   0x2810 //receive desc head
#define RDT   0x2818 //receive desc tail
#define RDTR  0x2820 //receive desc timer
#define RCTL  0x0100
//Global vars
//static int mod1=40;
//module_param(mod1,int,S_IRUSR | S_IWUSR);

//Interrupt
#define INTR_CAUSE_READ 0x000C0 //Interrupt Cause Read
#define INTR_CAUSE_SET  0x000C8 //Interrupt Cause Set
#define INTR_MASK_SET   0x000D0 //Interrupt Mask Set
#define INTR_MASK_CLEAR 0x000D8 //Interrupt Mask Clear
#define INTR_REG_TX     0x0001
#define INTR_REG_LSC	0x0005
#define INTR_REG_RXQ 	0x100000


static int blink_rate = 2;
module_param(blink_rate,int,S_IRUSR | S_IWUSR);
static struct timer_list my_timer;
static int open_close_status=0;
//static int *kbuffer;
static int on_off_state=1;

//static int pci_blinkdriver_probe(struct pci_dev *pdev, size_t len, loff_t *offset);
//static void pci_blinkdriver_remove(struct pci_dev *pdev);
static void my_callback(struct timer_list *list);

static struct class *my_func;

//Interrupt cookie
static struct cookie{
	struct pci_dev *pdev;
}irq;

//struct for char driver
struct mydev{
	struct cdev my_cdev;	//ref for cdev
	dev_t mydev_node;	//device node
	unsigned int syscall_val;
	unsigned long led_start_addr;
	dma_addr_t addr;
	struct e1000e_rx_desc *cpu_addr;
	void *hw_addr;
	struct work_struct srv_tsk;
	int tail;
	int head_tail;
}mydev;

//PCI struct
struct my_pcidev{
	struct pci_dev *pdev;
	void *mem_addr;
}my_pcidev;


//Buffer  information
struct buf_info{
	dma_addr_t phy_addr;
	void *mem;
}buf_info[16];

//E1000E receive descriptor
struct e1000e_rx_desc{
	__le64 buffer_addr; //address of the desc
	union{
		__le32 data;
		struct{
			__le16 length;
			u16 checksum;
		}flags;
	}lower;
	union{
		__le32 data;
		struct{
			u8 status;
			u8 error;
			__le16 special;
		}fields;
	}upper;
};

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

static void irq_work(struct work_struct *work)
{
	printk(KERN_INFO"entered work struct \n");
	//Sleep 0.5 secs
	msleep(500);
	printk(KERN_INFO"turn off both led\n");
	writel(0x0F0F,my_pcidev.mem_addr+0xE00);
	printk(KERN_INFO"Task is processing!!\n");
}


//Interrupt Handler
static irqreturn_t irq_handler(int irq, void *data){
	int i;
	u32 status;
	//Turn on both Green LED on
	printk(KERN_INFO"Received Interrupt\n");
	writel(0x0E0E0E0F,my_pcidev.mem_addr+0xE00);
	writel(0x10000,my_pcidev.mem_addr+INTR_MASK_SET);
	status = readl(my_pcidev.mem_addr+INTR_CAUSE_READ);
	status = status & 0x00FFFFFF;
	printk(KERN_INFO"Status:0x%x \n",status);
	for(i=0;i<16;i++)
		printk(KERN_INFO"data: 0x%x \t DD: 0x%x\n",mydev.cpu_addr[i].upper.data, mydev.cpu_addr[i].upper.fields.status);
	if(mydev.tail ==15)
		mydev.tail=0;
	else
	{
		mydev.cpu_addr[mydev.tail].upper.fields.status &= 0xFE;
		writel(mydev.tail, my_pcidev.mem_addr+RDT);
		mydev.tail++;
	}
	switch(status)
	{
		case INTR_REG_TX:
		case INTR_REG_LSC:
		case INTR_REG_RXQ:
			writel(status,mydev.hw_addr+INTR_MASK_CLEAR);
			schedule_work(&mydev.srv_tsk);
			break;
		default:
			printk(KERN_INFO"unknow IRQ\n");
			writel(0x4,my_pcidev.mem_addr+INTR_MASK_SET);
			return 0;
	}
	//clear inter bit
	writel(0x4,my_pcidev.mem_addr+INTR_MASK_SET);
	return 0;
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
	
	int i,temp;	
	unsigned long reg;
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
	
	my_pcidev.mem_addr=ioremap(mem_start,mem_len);
	if(my_pcidev.mem_addr < 0)
	{
		printk(KERN_ERR"ioremap failed\n");
		goto unregister_ioremap;
	}
	//if device is working
	mydev.led_start_addr=readl(my_pcidev.mem_addr+0xE00);
	printk(KERN_INFO"Initial val is=%lx\n",mydev.led_start_addr);
	
	//display in kernel
	dev_info(&pdev->dev,"LED_REG=%02X\n",mydev.syscall_val);

	//allocate physical memory and assign the starting addr to 
	//rx desc
	mydev.cpu_addr=dma_alloc_coherent(&pdev->dev,256,&mydev.addr,GFP_KERNEL);
	printk(KERN_INFO"mydev.cpuaddr=%lx\n",mydev.cpu_addr);
	printk(KERN_INFO"mydev.addr=%llx",mydev.addr);
	
	reg=(mydev.addr>>32)&0xFFFFFFFF;
	printk(KERN_INFO"mmydev.addr=%llx\n",mydev.addr);

	printk(KERN_INFO"Higher:0x%lx\n",reg);
	writel(reg,my_pcidev.mem_addr+RDBAH);

	reg=mydev.addr & 0xFFFFFFFF;
	printk(KERN_INFO"Lower: 0x%lx \n",reg);
	writel(reg, my_pcidev.mem_addr+RDBAL);

	//16 descriptor
	for(i=0;i<16;i++){
		//make zero in the buffer memory
		buf_info[i].mem=kzalloc(2048, GFP_KERNEL);
		printk(KERN_INFO"buf_info[i].mem=%p\n",buf_info[i].mem);
		if(!buf_info[i].mem){
			return -ENOMEM;
		}
		buf_info[i].phy_addr =dma_map_single(&pdev->dev,buf_info[i].mem,2048,DMA_TO_DEVICE);
		printk(KERN_INFO"phy_addr=%d\n",buf_info[i]);
		if(!buf_info[i].phy_addr){
			 return -ENOMEM;
		}
	}
	temp=readl(my_pcidev.mem_addr+RDLEN);
	printk(KERN_INFO"hw_addr+RDLEN=%p\n",my_pcidev.mem_addr+RDLEN);
	temp=temp&0x0;
	temp=temp|0x2000;
	writel(temp,my_pcidev.mem_addr+RDLEN);
	printk(KERN_INFO"hwaddr+RDLEN after calc=%p\n",my_pcidev.mem_addr+RDLEN);
	
	printk(KERN_INFO"ring desc created\n");
	printk(KERN_INFO"physical address values \n");
	
	for(i=0;i<16;i++){
		printk(KERN_INFO"addr:0x%x\n",(int)buf_info[i].phy_addr);
	}
	printk(KERN_INFO"filling descriptor\n");
	
	for(i=0;i<16;i++){
		mydev.cpu_addr[i].buffer_addr=buf_info[i].phy_addr;
		printk(KERN_INFO"address of buffer- 0x%x \n",(int)mydev.cpu_addr[i].buffer_addr);

	}

	//Enable receiving
	temp=readl(my_pcidev.mem_addr+0x0004);
	temp = temp | 0x40;
	printk(KERN_INFO"enable: 0x%x\n",temp);
	writel(temp,my_pcidev.mem_addr + 0x0004);
	temp = readl(my_pcidev.mem_addr+RCTL); 
	temp = temp | 0x10;
	writel(temp,my_pcidev.mem_addr+RCTL);

	//Set tail desc
	writel(15,my_pcidev.mem_addr+RDT);
	temp=readl(my_pcidev.mem_addr+RCTL);
	temp=temp|0x2;
	writel(temp,my_pcidev.mem_addr+RCTL);

	//Set Enable receive enable RCTL
	writel(0x100000,my_pcidev.mem_addr+INTR_CAUSE_SET);
	writel(0x100000,my_pcidev.mem_addr+INTR_MASK_SET);

	//request irq
	request_irq(pdev->irq,irq_handler,0,"hw6_interrupt",NULL);
	dev_info(&pdev->dev,"Interrupt request successful \n");
	printk(KERN_INFO"value:0x%x\n",readl(my_pcidev.mem_addr+INTR_MASK_SET));
	//work queue
	INIT_WORK(&mydev.srv_tsk,irq_work);
	

	return 0;
unregister_ioremap:
	iounmap(my_pcidev.mem_addr);
unregister_selected_regions:
	pci_release_selected_regions(pdev,pci_select_bars(pdev,IORESOURCE_MEM));
	return -1;	
}

//pci_remove
static void pci_remove(struct pci_dev *pdev){
	int i;
	struct my_pcidev *pe=pci_get_drvdata(pdev);
	cancel_work_sync(&mydev.srv_tsk);
	//Free ring desc
	for(i=0;i<16;i++)
		dma_unmap_single(&pdev->dev,buf_info[i].phy_addr,2048,DMA_TO_DEVICE);
	dma_free_coherent(&pdev->dev,256,mydev.cpu_addr,mydev.addr);
	free_irq(pdev->irq,(void *)&irq);
	iounmap(pe->mem_addr);
	kfree(pe);
	printk(KERN_INFO"removing PCI driver \n");
	//iounmap(my_pcidev.mem_addr);
	pci_release_selected_regions(pdev,pci_select_bars(pdev,IORESOURCE_MEM));
	pci_disable_device(pdev);
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
	printk(KERN_INFO"device created function successful\n");
	

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
