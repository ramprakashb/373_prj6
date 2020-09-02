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
#include<linux/netdevice.h>
#include<linux/init.h>
#include<linux/timekeeping.h>
#include<linux/delay.h>
#include<linux/jiffies.h>

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
#define LEDS_ON (LED0_ON | LED1_ON | LED2_ON | LED3_ON)
//Control Register
#define CTRL 0x00000

//Number of descriptors
#define NUM_DESC 0b00000001

//Receive Descriptor address
#define REC_CNTRL 0x00100	//Receive control
#define REC_DESC_LOW 0x2800	//receive desc low
#define REC_DESC_HIGH 0x2804	//receive desc high
#define REC_DESC_LEN 0x2808	//receive desc length
#define REC_DESC_HEAD 0x2810	//receive desc head 
#define REC_DESC_TAIL 0x2818	//receive desc tail

//Interrupt 
#define INTR_CA_READ 0x000C0	//Interrupt Cause Read 
#define INTR_CA_SET 0x000C8	//Interrupt Cause Set
#define INTR_MSK_SET 0x000D0	//Interrupt Mask set
#define INTR_MSK_CLR 0X000D8	//Interrupt Mask clear

//LEDs
#define LED_MODE_ON 0xF
#define LED_MODE_OFF 0xE


static int blink_rate = 2;
module_param(blink_rate,int,S_IRUSR | S_IWUSR);
static struct timer_list my_timer;
static int open_close_status=0;
static int on_off_state=1;
static struct timer_list blinktimer;

//struct for char driver
static struct my_chardev_struct{
	struct cdev my_cdev;	//ref for cdev
	dev_t mydev_node;	//device node
	unsigned int syscall_val;	//
	unsigned long led_start_addr;
	void *hw_addr;	//
	dma_addr_t addr;	//
	int tail;		//
	int head_tail;		//
	struct e1000e_rx_desc *cpu_addr;	//
	struct work_struct service_task;	//
}mydev;


union RCTL_set{
	unsigned int set;
	struct bits{
		int R1:1;
		int EN : 1;
		int SBP : 1;
		int UPE : 1;
		int MPE : 1;
		int LPE : 1;
		int LBM : 2;
		int RDMTS : 2;
		int R2 : 2;
		int MO : 2;
		int R3: 2;
		int BAM :1;
		int BSIZE : 2;
		int VFE : 1;
		int CFIEN : 1;
		int CFI : 1;
		int R4 : 1;
		int DPF : 1;
		int PMCF : 1;
		int R5 : 1;
		int BSEX : 1;
		int SECRC : 1;
		int R6 : 5;
	}bits;
}RCTL_SET;

static struct class *my_func;

//PCI struct
static struct my_pcidev{
	struct pci_dev *pdev;
	void *mem_addr;
	struct cdev cdev;
	struct work_struct task;
}my_pcidev;

//Buffer
struct desc_ring{
	dma_addr_t addr;
	void *mem;
}desc_ring[16];

//Receive Descriptors
struct rx_desc{
	__le64 buffer_addr; //Address of desc data buffer
	union{
		__le32 data;
		struct{
			__le16 length; //Data buffer length
			u16 cso;
		}flags;
	}lower;
	union{
		__le32 data;
		struct {
			u8 status; //descriptor status
			u8 error;  //Checksum start
			__le16 vlan;
		}field;
	}upper;
};

static int flag;

//blink function
void blinkLED(unsigned long flag){
	if(blink_rate <= 0){	
		printk(KERN_ERR"Error!\n");
		return;
	}
	if(mydev.syscall_val = 0x7844E){
		flag=0;
		mydev.syscall_val = 0x7840E;
		printk(KERN_ERR"LED is off %x, flag=%lu\n",mydev.syscall_val,flag);
		writeb(mydev.syscall_val,(mydev.hw_addr+0xe00));
		mod_timer(&my_timer,HZ/blink_rate+jiffies);
	}
	else
	{
		flag=1;
		mydev.syscall_val=0x7844E;
		printk(KERN_ERR"LED is on ->%x,flag=%lu\n",mydev.syscall_val,flag);
		writeb(mydev.syscall_val,(mydev.hw_addr+0xe00));
		mod_timer(&my_time,HZ/blink_rate+jiffies);
	}
}

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

void init_reg_val(void){
	RCTL_SET.bits.R1 =0;
	RCTL_SET.bits.EN =0;
	RCTL_SET.bits.SBP=0;
	RCTL_SET.bits.UPE=1;
	RCTL_SET.bits.MPE=1;
	RCTL_SET.bits.LPE=1;
	RCTL_SET.bits.LBM=0;
	RCTL_SET.bits.RDMTS=0;
	RCTL_SET.bits.R2=0;
	RCTL_SET.bits.MO=0;
	RCTL_SET.bits.R3=0;
	RCTL_SET.bits.BAM=1;
	RCTL_SET.bits.BSIZE=0;
	RCTL_SET.bits.VFE=0;
	RCTL_SET.bits.CFIEN=0;
	RCTL_SET.bits.CFI=0;
	RCTL_SET.bits.R4=0;
	RCTL_SET.bits.DPF=0;
	RCTL_SET.bits.PMCF=0;
	RCTL_SET.bits.R5=0;
	RCTL_SET.bits.BSEX=0;
	RCTL_SET.bits.SECRC=0;
	RCTL_SET.bits.R6=0;
								
}


//File operations
//open file

static int file_open(struct inode *inode, struct file *file){
	printk(KERN_INFO"Kernel: File Opened \n");
	//timer init
	mod_timer(&blinktimer, (HZ/blink_rate)+jiffies);
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
	
	union body{
		unsigned int full;
		struct split{
			uint8_t head;
			uint8_t tail;
		}split;
	}body;

	unsigned int temp;

	printk(KERN_INFO"before readl, my_pcidev.mem_addr=%p",my_pcidev.mem_addr);

	printk(KERN_INFO"ADDED ADDR: %p",my_pcidev.mem_addr+0xE00);
	mydev.syscall_val= readl(my_pcidev.mem_addr+0xE00);
	printk(KERN_INFO"after readl, mypcidev.mem_addr+0xE00 ret val=%u",mydev.syscall_val);

	if(*offset >= sizeof(int)){
		return 0;
	}
	if(!buf){
		ret = -EINVAL;
		goto out;
	}
	//Read head 
	temp = readl(my_pcidev.mem_addr+REC_DESC_HEAD);
	printk(KERN_INFO"Head: %d\n",temp);
	body.split.head=(uint8_t)temp;
	temp=readl(my_pcidev.mem_addr+REC_DESC_TAIL);
	printk(KERN_INFO"Tail:%d\n",temp);
	body.split.tail=(uint8_t)temp;

	printk(KERN_INFO"Read memory address %u \n",mydev.syscall_val);
	if(copy_to_user(buf,&mydev.syscall_val,len)){
		printk(KERN_ERR"Copy to user error \n");
		ret = -EFAULT;
		goto out;
	}
	ret = sizeof(int);
	*offset+=len;
	printk(KERN_INFO"user got %d\n",body.full);
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


static irqreturn_t irq_handler(int irq, void* data){
	//Reset interrupts
	printk(KERN_INFO"entered into intr handler\n");
	writel(0xFFFF,my_pcidev.mem_addr+INTR_MSK_CLR);
	//Call scheduler
	printk(KERN_INFO"call scheduler\n");
	schedule_work(&mydev.work);
	printk(KERN_INFO"return from irq handler\n");
	return IRQ_HANDLED;
}

static void my_work(struct work_struct* work){
	printk(KERN_INFO"service task triggered!!!\n");
	if(readl(my_pcidev.mem_addr+INTR_CA_READ)|(1<<13)){
		printk(KERN_INFO"interrupt cause read");
	}
	//set interrupt
	writel((1<<13),my_pcidev.mem_addr+INTR_MSK_SET);
}


static int pci_probe(struct pci_dev *pdev,const struct pci_device_id *ent){
	int err;
	resource_size_t mem_start, mem_len;
	unsigned long barmask;
	
	printk(KERN_INFO"probe function called\n");
	
	//Initialize settings values for later writes
	init_reg_val();

	//prep work queue
	INIT_WORK(&mydev.work,my_work);


	//Set Low or High DMA
	err=dma_set_mask(&pdev->dev,DMA_BIT_MASK(64));
	if(err) {
		dev_err(&pdev->dev,"DMA Config failed:0x%x\n",err);
		pci_disable_device(pdev);
		return err;
	}

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
	//Read Current value LED from address LED
	mydev.syscall_val = readl(mydev.hw_addr+0xe00);
	
	//display it in kernel
	dev_info(&pdev_>dev,"LED_REG=0x%02x\n",mydev.syscall_val);

	//Allocate memory for PCI
	mydev.cpu_addr=dma_alloc_coherent(&pdev->dev,256,&mydev.addr,GFP_KERNEL);
	printk(KERN_INFO"mydev.cpuaddr=%lx\n",mydev.cpu_addr);
	printk(KERN_INFO"mydev.addr=%llx",mydev.addr);

	reg=(mydev.addr>>32)&0xFFFFFFFF;
	printk(KERN_INFO"Higher:0x%x\n",reg);
	writel(reg,mydev.hw_addr+REC_DESC_HIGH);

	reg=mydev.addr&0xFFFFFFFF;
	printk(KERN_INFO"lower:0x%x\n",reg);
	writel(reg, mydev.hw_addr+REC_DESC_LOW);

	//16 descriptors
	for(i=0;i<16;i++)
	{
		buf_info[i].mem=kzalloc(2048,GFP_KERNEL);
		if(!buf_info[i].mem){
			err = -ENOMEM;
			goto err_descriptor;
		}
		buf_info[i].addr = dma_map_single(&pdev->dev,buf_info[i].mem,2048,DMA_TO_DEVICE);
		if(!buf_info[i].addr){
		err=-ENOMEM;
		goto err_descriptor;
		}
	}

	//length of desc
	temp=readl(mydev.hw_addr+REC_DESC_LEN);
	temp=temp&0x0;
	temp=temp|0x2000;
	writel(temp,mydev.hw_addr+REC_DESC_LEN);

	printk(KERN_INFO"Ring desc created\n");
	printk(KERN_INFO"Physical addr, info\n");
	for(i=0;i<16;i++)	
		printk(KERN_INFO"ADDR:0x%x\n",(int)buf_info[i].addr);
	printk(KERN_INFO"filling desc");
	for(i=0;i<16;i++){
		mydev.cpu_addr[i].buffer_addr=buf_info[i].addr;
		printk(KERN_INFO"Addr of buffer:0x%x\n",(int)mydev.cpu_addr[i].buffer_addr);
	}
	
	//Receive val
	temp=readl(mydev.hw_addr+0x0004); //Read
	temp=temp|0x40;
	printk(KERN_INFO"Enable:0x%x",temp);
	writel(temp,mydev.hw_addr+0x0004); //Write
	temp=readl(mydev.hw_addr+REC_CNTRL); //Read
	temp = temp | 0x10; //Modify
	writel(temp,mydev.hw_addr+REC_CNTRL); //Write
	
	//Set tail descriptor
	writel(15,mydev.hw_addr+REC_DESC_TAIL); //Read
	temp=readl(mydev.hw_addr+REC_CNTRL);
	temp=temp|0x2;	
	writel(temp,mydev.hw_addr+REC_CNTRL); //Write

	//Set Enable Receive enable cntrl reg
	writel(0x100000,mydev.hw_addr+INTR_CA_SET);
	writel(0x100000,mydev.hw_addr+INTR_MSK_SET);
	
	//Request IRQ
	if(request_irq(pdev->irq,irq_hdlr,0,"hw_intr"),(void*)&irq));
		dev_info(&pdev->dev,"IRQ requested successfully\n");
	printk(KERN_INFO"Value:0x%x",readl(mydev.hw_addr+INTR_MSK_SET));

	//Work queue
	INIT_WORK(&mydev.service_task,work);

	printk(KERN_INFO"probe");
	return 0;
unregister_ioremap:
	iounmap(my_pcidev.mem_addr);
unregister_selected_regions:
	pci_release_selected_regions(pdev,pci_select_bars(pdev,IORESOURCE_MEM));
}



//pci_remove
static void pci_remove(struct pci_dev *pdev){
	printk(KERN_INFO"removing PCI driver \n");
	struct pes *pe=pci_get_drvdata(pdev);
	cancel_work_sync(&mydev.service_task);
	iounmap(my_pcidev.mem_addr);
	cancel_work_sync(&mydev.service_task);
	//Free ring desc
	for(i=0;i<16;i++)
		dms_unmap_single(&pdev,buf)info[i].addr,2048,DMA_TO_DEVICE);
	dma_free_coherent(&pdev->dev,256,mydev.cpu_addr,mydev.addr);
	free_irq(pdev->irq,(void *)irq);
	iounmap(pe->hw_addr);
	kfree(pe);	
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
	.name = "hw6_pcidriver",
	.id_table = pci_ids,
	.probe = pci_probe,
	.remove = pci_remove,
};

//Module Initialization
static int __init myfunc_init(void){
	int ret_pci;
	mydev.syscall_val = 40;
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
	printk(KERN_INFO"cdev_init\n");
	mydev.my_cdev.owner=THIS_MODULE;
	//Register PCI driver
	ret_pci=pci_register_driver(&my_pcidriver);
	printk(KERN_INFO"pci reg driver=%d\n",ret_pci);
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
	//Allocate memory for desc Rings
	//desc_ring=kmalloc(sizeof(struct desc_ring),GFP_DMA);

	desc_ring = dma_alloc_coherent(&pdev->dev,256,&mydev.addr,GFP_KERNEL);
	if(desc_ring == NULL){
		printk(KERN_ERR"Desc ring failed\n");
		kfree(desc_ring);
		return -1;
	}
	printk(KERN_INFO"Descritor ring:%p\n",desc_ring);

	timer_setup(&blinktimer,blinkLED,0);
	printk(KERN_INFO"Timer setup!!!\n");
	return 0;
}
void __exit myfunc_exit(void){
	pci_unregister_driver(&my_pcidriver);
	cdev_del(&mydev.my_cdev);
	unregister_chrdev_region(mydev.mydev_node,DEVCNT);
	printk(KERN_INFO"module unloaded\n");
	kfree(desc_ring);
	printk(KERN_INFO"\n---Char dev module unloaded---\n");
}



MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ram");
MODULE_VERSION("0.1");
module_init(myfunc_init);
module_exit(myfunc_exit);
