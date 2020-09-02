
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/pci.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/usb.h>
#include <linux/slab.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/workqueue.h>

#define DEVCNT 1
#define DEVNAME "hw6_kernel"

#define PCI_REG_LEDS 0xe00
#define PCI_REG_CTRL 0x0004


//LED commands
#define LED0_ON 	0xe
#define LED0_OFF	0X4e
#define LED1_ON 	0xe00
#define LED1_OFF 	0x4e00
#define LED2_ON 	0xe0000
#define LED2_OFF 	0x4e0000
#define LED3_ON 	0xe000000
#define LED3_OFF 	0x4E000000
#define LEDS_OFF 	(LED0_OFF | LED1_OFF | LED2_OFF | LED3_OFF)
#define LEDS_ON 	(LED0_ON | LED1_ON | LED2_ON | LED3_ON)


// Define for interrupt
#define INTR_CA_READ   0x000C0 	//INTERRUPT CAUSE READ
#define INTR_CA_SET    0x000C8	//INTERRUPT CAUSE SET
#define INTR_MSK_SET   0x000D0	//INTERRUPT MASK SET
#define INTR_MSK_CLR   0x000D8	//INTERRUPT MASK CLEAR


// Define Receive Reg
#define REC_CNTRL_REG 0x0100 	//RECEIVE CONTROL REG
#define REC_DESC_LEN  0x2808	//RECEIVE DESC TAIL
#define REC_DESC_HEAD 0x2810	//RECEIVE DESC HEAD
#define REC_DESC_TAIL 0x2818	//RECEIVE DESC LENGTH
#define REC_DESC_HIGH 0x2804	//RECEIVE DESC HIGH
#define REC_DESC_LOW  0x2800	//RECEIVE DESC LOW

#define IRQ_REG_RXQ   0x100000

static struct timer_list my_time;
static int flag;
static struct class *my_func;

// Create Parameter delay with default = 2
static int blink_rate = 2;
module_param(blink_rate,int,S_IRUSR | S_IWUSR);

static struct interrupt
{
	struct pci_dev *pdev;
}irq;

// Struct mydev
 static struct my_chardev_struct {
	struct cdev cdev;
	void *hw_addr;    
	u32 syscall_val;  
	dma_addr_t addr;	
	int tail; 
	int head_tail;
	dev_t mydev_node;	//device node
	struct e1000e_rx_desc  *cpu_addr;
	struct work_struct service_task;
	struct timer_list; 
} mydev; 

//Timer Init

void timer_init(void){
	//timer_setup(&my_time, my_callback, 0);
	printk(KERN_INFO"Timer creaeted successfully");
	mydev.syscall_val = 1000/blink_rate;
	printk("1/blinkrate=%d\n",mydev.syscall_val);
	//jiffies are eqal to sec(1/hz)
	mod_timer(&my_time,jiffies+msecs_to_jiffies(mydev.syscall_val));
	printk(KERN_INFO"mod timer executed successfully\n");
}

// Struct buffer infor
struct buf_info
{
	dma_addr_t addr;
	void *mem;
}buf_info[16];

// Receive Descriptor
struct e1000e_rx_desc
{
	__le64 buffer_addr; 		// Address of the descriptor's data buffer 
	union
	{
		__le32 data;
		struct
		{
			__le16 length; // Data buffer length 
			u16 cso; 	// Checksum 
		} flags;
	} lower;
	union 
	{
		__le32 data;
		struct
		{
			u8 status; 	// Descriptor status
			u8 error; 	// Checksum  
			__le16  Vlan;
		} field;
	} upper;
};

//PCI device functions
//PCI device id
static const struct pci_device_id pci_ids[]={
	{ PCI_DEVICE(0x8086, 0x100e)},
	{}	
};






// Struct PCI struct
static struct my_pcidev {
	struct pci_dev *pdev;
	void *hw_addr;
	struct cdev cdev;
	struct work_struct task;
}my_pcidev;




static void irq_work(struct work_struct *work)
{
	printk(KERN_INFO "In work queue :)\n");
	printk(KERN_INFO "Start and sleep 0.5 secs \n"); 
	msleep(500);
	printk(KERN_INFO "Turn off both green LEDs");
	writel(0x0F0F0F, mydev.hw_addr + PCI_REG_LEDS);
	printk(KERN_INFO "intr work complete \n");
}


// REQUEST IRQ
static irqreturn_t irq_hdlr(int irq,void *data)
{
	int i;
	u32 status;

        // Turn on both Green LED when go to interrupt handler
	printk(KERN_INFO "Received Interrupt\n");
	writel(0x4E4E0F,mydev.hw_addr + PCI_REG_LEDS);
	writel(0x100000,mydev.hw_addr + INTR_MSK_SET);
	status = readl(mydev.hw_addr+INTR_CA_READ);
	status = status & 0x00FFFFFF; 
	printk(KERN_INFO "Status: 0x%x \n",status);
	for(i=0;i<16;i++)
	printk(KERN_INFO "Data: 0x%x    DD-bit: 0x%x\n",
	mydev.cpu_addr[i].upper.data,mydev.cpu_addr[i].upper.field.status &0x1);
	
	if(mydev.tail==15)
		mydev.tail=0;
	else
	{
		mydev.cpu_addr[mydev.tail].upper.field.status &= 0xFE;
		writel(mydev.tail,mydev.hw_addr + REC_DESC_TAIL);
		mydev.tail++;
	}

        // interrupt signal
	switch(status)
	{
	 
		case 0x0001:
		case 0x0005:
		case 0x100000: 
			writel(status,mydev.hw_addr + INTR_MSK_CLR);
			schedule_work(&mydev.service_task);
			break;
		default:
			printk(KERN_INFO "Unknow IRQ \n");
			writel(0x4,mydev.hw_addr + INTR_MSK_SET);
			return IRQ_NONE;
		
	}

        // Clear interrupt bit
	writel(0x4,mydev.hw_addr+ INTR_MSK_SET);
	return IRQ_HANDLED;
}  



static void timer_cb(struct timer_list *time)
{
	if(blink_rate <=0)
	{
	 printk(KERN_ERR "Error! \n");
	 return;
	}

	if(mydev.syscall_val==0x7844E)
	{	
		flag=0;
		mydev.syscall_val = 0x7840E;
		printk(KERN_ERR "LED is off :( -> %x, flag : %u\n",mydev.syscall_val,flag);
		writeb(&mydev.syscall_val,(mydev.hw_addr + PCI_REG_LEDS));
		mod_timer(&my_time,(HZ/blink_rate + jiffies));
	}
	else
	{	
		flag=1;
		mydev.syscall_val = 0x7844E;
		printk(KERN_ERR "LED is on :) ->%x, flag : %u\n",mydev.syscall_val,flag);
		writeb(mydev.syscall_val,(mydev.hw_addr+ PCI_REG_LEDS));
		mod_timer(&my_time,(HZ/blink_rate + jiffies));
	}
}

static int pci_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct my_pcidev *mpdev;
	printk(KERN_INFO"probe function called\n");
	resource_size_t mem_start, mem_len;
	unsigned long barmask;		
	int err;
	int i;
	unsigned int reg;
	int temp;
	u32 ioremap_len;

	 // set up for high or low DMA
	err = dma_set_mask(&pdev->dev, DMA_BIT_MASK(64));
	if (err) {
		dev_err(&pdev->dev, "DMA configuration failed: 0x%x\n", err);
		pci_disable_device(pdev);
	}

	//get base address
	barmask = pci_select_bars(pdev,IORESOURCE_MEM);
	printk(KERN_INFO"barmask=%lx",barmask);
	//get the pci selected regions
	if(pci_request_selected_regions(pdev,barmask,"pcidriver")){
		printk(KERN_ERR"selected req region failed\n");
		return err;
	}
	pci_set_master(pdev);
	mpdev = kzalloc(sizeof(*mpdev),GFP_KERNEL);
	if(!mpdev){
		err = -ENOMEM;
		pci_release_selected_regions(pdev, pci_select_bars(pdev, IORESOURCE_MEM));
	}	
	mpdev->pdev = pdev;
	pci_set_drvdata(pdev,mpdev);
	
	//map device memory
	ioremap_len  = min_t(int, pci_resource_len(pdev,0),32768);

	mem_start=pci_resource_start(pdev,0);
	mem_len = pci_resource_len(pdev,0);
	printk(KERN_INFO"mem start:%lx\n",(unsigned long)mem_start);
	printk(KERN_INFO"mem len:%lx\n",(unsigned long)mem_len);
	//pci_set_master(pdev);
	
	
	mydev.hw_addr=ioremap(mem_start,mem_len);
	if(!mydev.hw_addr)	
	{
		printk(KERN_ERR"ioremap failed\n");
		iounmap(my_pcidev.hw_addr);
	}

	mpdev->hw_addr=ioremap(mem_start,mem_len);

	printk(KERN_INFO"ioremap output=%p\n",my_pcidev.hw_addr);	 
	//Read current value LED from address LED
	mydev.syscall_val = readl(mydev.hw_addr + 0xe00);
	printk(KERN_INFO"MydevSyscall_val=%d\n",mydev.syscall_val);
	// Display it in Kernel
	dev_info(&pdev->dev, "LED_REG = 0x%02x\n",mydev.syscall_val);

	mydev.cpu_addr= dma_alloc_coherent(&pdev->dev,256,&mydev.addr,GFP_KERNEL);

	printk(KERN_INFO"mydev.cpu_addr = %lu\n",mydev.cpu_addr);	
	printk(KERN_INFO"mydev.addr=%llu\n",mydev.addr);

	reg = (mydev.addr >> 32) & 0xFFFFFFFF;
	
	printk(KERN_INFO"reg=%llu\n",reg);
	printk(KERN_INFO"(mydev.addr>>32)=%llu\n",mydev.addr>>32);
	printk(KERN_INFO"(mydev.addr>>32)&0xFFFFFFFF=%llu\n",((mydev.addr>>32)&0xFFFFFFFF));

	printk(KERN_INFO "Higher: 0x%x \n", reg);
	writel(reg, mydev.hw_addr + REC_DESC_HIGH);

	reg = mydev.addr &0xFFFFFFFF;
	printk(KERN_INFO "Lower: 0x%x \n", reg);
	writel(reg,mydev.hw_addr+ REC_DESC_LOW);
	
        // 16 descriptors
	for(i=0; i<16; i++)
	{
		buf_info[i].mem= kzalloc(2048,GFP_KERNEL);
		if(!buf_info[i].mem)
		{
			err = -ENOMEM;
		}
		buf_info[i].addr = dma_map_single(&pdev->dev,buf_info[i].mem,2048,DMA_TO_DEVICE);
		if(!buf_info[i].addr)
		{
			err = -ENOMEM;
		} 
	}

// Set the Length
	temp = readl(mydev.hw_addr + REC_DESC_LEN);
        temp = temp & 0x0;
	temp = temp | 0x2000;
        writel(temp,mydev.hw_addr + REC_DESC_LEN);

	printk(KERN_INFO "Ring Descriptor created! :) \n");
	printk(KERN_INFO "Physical Addr. Info \n");
	for(i=0;i<16;i++)
		printk(KERN_INFO "Addr: 0x%x \n",(int)buf_info[i].addr);
	

	printk(KERN_INFO "Filling up descriptor buffer:");
	for(i=0;i<16;i++)
	{
		mydev.cpu_addr[i].buffer_addr= buf_info[i].addr;
		printk(KERN_INFO "Address of buffer: 0x%x \n",(int)mydev.cpu_addr[i].buffer_addr);
	
	}
	
	// Enable receiving
	temp = readl(mydev.hw_addr + 0x0004);
	temp = temp | 0x40;
	printk(KERN_INFO "Enable: 0x%x",temp);
	writel(temp,mydev.hw_addr + 0x0004); 
	temp = readl(mydev.hw_addr + REC_CNTRL_REG); 
	temp = temp | 0x10 ; // Modify
	writel(temp,mydev.hw_addr + REC_CNTRL_REG); 

	// Set tail descriptor
	writel(15,mydev.hw_addr + REC_DESC_TAIL);
	temp = readl(mydev.hw_addr + REC_CNTRL_REG);
        temp = temp | 0x2 ; // Modify
        writel(temp,mydev.hw_addr + REC_CNTRL_REG); 

        // Set Enable Receive enable RCTL
	writel(0x100000,mydev.hw_addr + INTR_CA_SET);
	writel(0x100000,mydev.hw_addr + INTR_MSK_SET);

        // Request IRQ
	if (request_irq(pdev->irq,irq_hdlr,0,"hw_intr",(void *)&irq));
		dev_info(&pdev->dev,"IRQ requested successfully \n");
	printk(KERN_INFO "value: 0x%x",readl(mydev.hw_addr+INTR_MSK_SET));

        // Work queue
	INIT_WORK(&mydev.service_task,irq_work);
	return 0;

}

static void pci_remove(struct pci_dev *pdev)
{
	int i;
	struct my_pcidev *pe = pci_get_drvdata(pdev);
	cancel_work_sync(&mydev.service_task);

        // Free ring descriptor
	for(i=0;i<16;i++)
		dma_unmap_single(&pdev->dev,buf_info[i].addr,2048,DMA_TO_DEVICE);
	dma_free_coherent(&pdev->dev,256,mydev.cpu_addr,mydev.addr);
	free_irq(pdev->irq,(void *)&irq);	
	iounmap(pe->hw_addr);
	kfree(pe);
	pci_release_selected_regions(pdev, pci_select_bars(pdev, IORESOURCE_MEM));
	pci_disable_device(pdev);	
}

static struct pci_driver my_pcidriver = {
	.name     = "hw6_pcidriver",
	.id_table = pci_ids,
	.probe    = pci_probe,
	.remove   = pci_remove,
};

// Device Driver
static dev_t mydev_node;

/* this shows up under /sys/modules/HW6/parameters */
static int exam = 15;
module_param(exam, int, S_IRUSR | S_IWUSR);
/* this doesn't appear in /sys/modules */
static int exam_nosysfs = 15;
module_param(exam_nosysfs, int, 0);

static int file_open(struct inode *inode, struct file *file)
{
	printk(KERN_INFO"Kernel: File Opened \n");
	//timer init
	timer_init();
	return 0;
}


//release file

static int file_release(struct inode *inode, struct file *file){
	printk(KERN_INFO"Kernel: File released\n");
	//open_close_status = 0;
	del_timer_sync(&my_time);
	return 0;
}


static ssize_t file_read(struct file *file, char __user *buf,
                             size_t len, loff_t *offset)
{
	int ret, temp;
	printk(KERN_INFO"before readl=%p",my_pcidev.hw_addr);

	printk(KERN_INFO"ADDED ADDR: %p",my_pcidev.hw_addr+0xE00);
	mydev.syscall_val= readl(my_pcidev.hw_addr+0xE00);
	printk(KERN_INFO"after readl, mypcidev.mem_addr+0xE00 ret val=%u",mydev.syscall_val);
	if(!buf){
		ret = -EINVAL;
		goto out;
	}

	printk(KERN_INFO"Read memory address %u \n",mydev.syscall_val);
	printk(KERN_INFO"before copy to user %u \n",mydev.head_tail );
	if(copy_to_user(buf,&mydev.head_tail,sizeof(int))){
		printk(KERN_ERR"Copy to user error \n");
		ret = -EFAULT;
		goto out;
	}
	ret = sizeof(int);
	*offset += len;
	mydev.head_tail = readl(mydev.hw_addr + REC_DESC_HEAD); // Read HEAD's value
	mydev.head_tail = mydev.head_tail << 16; // 16 bit left shift
	temp = readl(mydev.hw_addr + REC_DESC_TAIL); // Read TAIL's value
	mydev.head_tail = mydev.head_tail | temp; // 16 bit right shift	

out:
	return ret;
}

static ssize_t file_write(struct file *file, const char __user *buf,
                              size_t len, loff_t *offset)
{
	// Have local kernel memory ready
	char *kern_buf;
	int ret;
	
	// Make sure our user isn't bad
	if (!buf) {
		ret = -EINVAL;
		goto out;
	}

	// Get memory to copy into
	kern_buf = kmalloc(len, GFP_KERNEL);

	// Make sure the memory is good to go
	if (!kern_buf) {
		ret = -ENOMEM;
		goto out;
	}

	// Copy from the user buffer
	if (copy_from_user(kern_buf, buf, len)) {
		ret = -EFAULT;
		goto mem_out;
		goto out;
	}

	// Update syscall_val value
	if (kstrtoint(kern_buf,10,&blink_rate))
        {
		ret = -ERANGE;
		goto mem_out;
	}
	ret = len;

	// Print syscall_val's new value
	printk(KERN_INFO "The new value for blink_rate is -> \"%d \n", blink_rate);
	
mem_out:
	kfree(kern_buf);
out:
	return ret;
}

// File operations for this device
static struct file_operations mydev_fops = {
	.owner = THIS_MODULE,
	.open = file_open,
	.read = file_read,
	.write = file_write,
	.release = file_release,
};

static int __init HW6_init(void)
{
	int ret_pci;
	printk(KERN_INFO "Module loaded\n");
	
	/* Dynamically allocalte a char driver*/
	if (alloc_chrdev_region(&mydev_node, 0, DEVCNT, DEVNAME)) {
		printk(KERN_ERR "alloc_chrdev_region() failed!\n");
		return -1;
	}
	printk(KERN_INFO "Allocated %d devices at major: %d\n", DEVCNT,
	       MAJOR(mydev_node));

	if((my_func = class_create(THIS_MODULE,DEVNAME))==NULL){
		printk(KERN_ERR"class create failed\n");
		class_destroy(my_func);
	}
	printk(KERN_INFO"return my_func");

	/* Initialize the character device and add it to the kernel */
printk(KERN_INFO"Allocated %d devices at major %d\n",DEVCNT, MAJOR(mydev.mydev_node));
	cdev_init(&mydev.cdev, &mydev_fops);
	mydev.cdev.owner = THIS_MODULE;
	printk(KERN_INFO"cdev init \n");
	ret_pci = pci_register_driver(&my_pcidriver);
	printk(KERN_INFO"pci register driver \n");
	if(ret_pci<0){
		pci_unregister_driver(&my_pcidriver);
		printk(KERN_ERR"PCI driver registration failed\n");
		return -1;
	}

	if (cdev_add(&mydev.cdev, mydev_node, DEVCNT)) {
		printk(KERN_ERR "cdev_add() failed!\n");
		/* clean up chrdev allocation */
		unregister_chrdev_region(mydev_node, DEVCNT);
		return -1;
	}
	

	//Creating device node automatically
	//class_device_create(my_func,NULL,mydev.devnode,NULL,"autodrv");

	if(device_create(my_func,NULL,mydev.mydev_node,NULL,DEVNAME)==NULL){
		printk(KERN_ERR"device create failure\n");
		class_destroy(my_func);
	}	

	// Setup the timer
	timer_setup(&my_time, timer_cb,(unsigned long)&flag);
	
	

	// Initialize the tail value
	//mydev.tail = 0;
	return 0;
}

static void __exit HW6_exit(void)
{
	pci_unregister_driver(&my_pcidriver);

	//destroy the cdev
	cdev_del(&mydev.cdev);

	//clean up the devices
	unregister_chrdev_region(mydev_node, DEVCNT);
	
	//Unload Timmer
	del_timer_sync(&my_time);
	printk(KERN_INFO "%s Module is unloaded\n", my_pcidriver.name);
}
MODULE_AUTHOR("Ram");
MODULE_LICENSE("GPL");
module_init(HW6_init);
module_exit(HW6_exit);
