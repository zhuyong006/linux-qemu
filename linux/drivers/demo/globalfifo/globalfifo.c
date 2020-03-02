
#include<linux/module.h>  

#include<linux/types.h>  

#include<linux/fs.h>  

#include<linux/errno.h>  

#include<linux/mm.h>  

#include<linux/sched.h>  

#include<linux/init.h>  

#include<linux/cdev.h>  

#include<asm/io.h>  

#include<asm/uaccess.h>  

#include<linux/poll.h>  

#include <linux/slab.h>

#include <linux/ioctl.h>

  

#define GLOBALFIFO_SIZE  10  /*ȫ��fifo���10�ֽ�*/  

#define FIFO_CLEAR 0X1       /*��0ȫ���ڴ�ĳ���*/  

#define GLOBALFIFO_MAJOR 100 /*���豸��*/  

  

static int globalfifo_major = GLOBALFIFO_MAJOR;  

  

/*globalfifo�豸�ṹ��*/  

struct globalfifo_dev{  

	struct cdev cdev;                   /*cdev�ṹ��*/  

	unsigned int current_len;           /*fifo��Ч���ݳ���*/  

    unsigned char mem[GLOBALFIFO_SIZE]; /*ȫ���ڴ�*/  //�ں˵Ĺ���������

    struct semaphore sem;               /*���������õ��ź���*/  

    wait_queue_head_t r_wait;           /*�������õĵȴ�����,�ں�˫��ѭ������*/  

    wait_queue_head_t w_wait;           /*����д�õĵȴ�����ͷ*/  

};  

  

struct globalfifo_dev *globalfifo_devp; /*�豸�ṹ��ָ��*/  

  

/*globalfifo������*/  //cat /dev/xxx

static ssize_t globalfifo_read(struct file *filp, 

	char __user *buf, size_t count, loff_t *ppos)  

{  

    int ret;  

    struct globalfifo_dev *dev = filp->private_data;  //private_dataȫ�ֱ���

    DECLARE_WAITQUEUE(wait, current);  //���嵱ǰ���̵ĵȴ�����wait��currentָ��ָ��ǰ�����еĽ���

  

    down(&dev->sem);                     /*����ź���*/  

    add_wait_queue(&dev->r_wait, &wait); /*������ȴ�����ͷ ���ں�*/ //��wait��ӵ��ȴ�����ͷr_waitָ��ĵȴ����������У����������Ѿ�˯���ˣ�����Ҫ���Ⱥ����ĵ���

  

    /*�ȴ�FIFO �ǿ�*/ //�������������mem�����ݳ���Ϊ0����Ӧ�������ý���

    if(dev->current_len == 0){  

        if(filp->f_flags & O_NONBLOCK){   /*�������Ϊ �������� �豸�ļ�*/  

            ret = -EAGAIN;  //�ٽ���һ�ζ�����

            goto out;  

        }  

        __set_current_state(TASK_INTERRUPTIBLE); /*�ı����״̬Ϊ˯��*/  

        up(&dev->sem);                           /*�ͷ��ź���*/  

  

        schedule();                              /*������������ִ��*/  //��ʱ�����̲Ż�������˯�ߣ�ֱ����д���̻��ѡ���˯��;�У�����û��������̷������źţ���ôҲ�ỽ��˯�ߵĽ���

        if(signal_pending(current)){             /*�������Ϊ�źŻ���*/ //��Ϊ�ǵ��ȳ�ȥ������״̬��ǳ��˯�ߣ����������п������ź� 

            ret = -ERESTARTSYS;  //��ʾ�źź���������Ϻ�����ִ���źŴ�����ǰ��ĳ��ϵͳ����

            goto out2;  

        }  

        down(&dev->sem);  //�����ź���down��up��ֹ�������ͬʱ���ʹ�������mem

    }  

  

    /*�������û��ռ�*/  

    if(count > dev->current_len)  //�����ǰ�������ݴ���fifo����Ч���ݳ���

        count = dev->current_len;  

    if(copy_to_user(buf, dev->mem, count)){  //������to from count���ɹ�����0��ʧ���Ƿ��ػ�û�п������û��ռ���ֽ���

        ret = -EFAULT;  

        goto out;  

    }else{  

        memcpy(dev->mem, dev->mem + count, dev->current_len - count);/*����ǰ��*/  //memcpy��(dev->mem + count)��ʼ��(dev->current_len - count)�ֽڵ������ƶ����������ʼ�ĵط�

        dev->current_len -= count; /*��Ч���ݳ��ȼ���*/  

        printk(KERN_INFO"read %ld bytes(s),current_len:%d\n",count, dev->current_len);  

  

        wake_up_interruptible(&dev->w_wait); /*����д�ȴ�����*/	//�Ѿ��������ݣ���Ҫ����д����������д����

        ret = count;  

    }  

out:  

    up(&dev->sem); /*�ͷ��ź���*/  

out2:  

    remove_wait_queue(&dev->w_wait, &wait); /*�����ĵȴ�����ͷ�Ƴ�*/  

    set_current_state(TASK_RUNNING);  

    return ret;  

}  

  

/*globalfifo д����*/  //echo " " > /dev/xxx

static ssize_t globalfifo_write(struct file *filp, 

	const char __user *buf, size_t count, loff_t *ppos)  

{  

    struct globalfifo_dev *dev = filp->private_data;  

    int ret;  

    DECLARE_WAITQUEUE(wait, current);    /*����ȴ�����*/    

    down(&dev->sem);                     /*����ź���*/  

    add_wait_queue(&dev->w_wait, &wait); /*����д�ȴ�����ͷ*/  

  

    /*�ȴ�FIFO����*/  //������������������ݳ��ȵ���fifo�Ĵ�С����ʾ�Ѿ����ˣ���Ӧ������

    if(dev->current_len == GLOBALFIFO_SIZE){  

        if(filp->f_flags & O_NONBLOCK){   

			/*������̷������򿪵��ļ�*/  

            ret = -EAGAIN;  

            goto out;  

        }  

  

        __set_current_state(TASK_INTERRUPTIBLE); /*�ı����״̬Ϊ˯��*/  

        up(&dev->sem);                     /*�ͷ��ź���*/  

  

        schedule();                         /*������������ִ��*/  

        if(signal_pending(current)){  

                                            /*�������Ϊ�źŻ���*/  

            ret = -ERESTARTSYS;  

            goto out2;  

        }  

        down(&dev->sem);                    /*����ź���*/  

    }  

  

    /*���û��ռ俽�����ݵ��ں˿ռ�*/  

    if(count > GLOBALFIFO_SIZE - dev->current_len){	//���fifo�Ĵ�С������Ч�ڴ�ĳ��ȣ����´���д  

        /*���Ҫ���������ݴ��� ʣ����Ч�ڴ泤��   

         *�� ֻ������� ��װ�µĳ���*/  

        count = GLOBALFIFO_SIZE - dev->current_len;  //count�����´���д�����ݴ�С

    }  

    if(copy_from_user(dev->mem + dev->current_len, buf, count)){  //����to��from��count��to��(dev->mem) + (dev->current_len)����Ϊд�������ˣ����Ե�ǰ�Ĺ�����������Ҫ��λ

        ret = -EFAULT;  

        goto out;  

    }else {  

        dev->current_len += count;  //��Ч�����мӵ�ǰ�����ݵĳ���

        printk(KERN_INFO"written %ld bytes(s), current_len: %d\n",count, dev->current_len);  

  

        wake_up_interruptible(&dev->r_wait); /*���Ѷ��ȴ�����*/ //д�����ݣ��ǿ϶�Ҫ���Ѷ����н��ж��� 

        ret = count;  

    }  

    out:  

        up(&dev->sem); /*�ͷ��ź���*/  //�ͷ��ź����������̻����ź������ͷŶ�����

    out2:  

        remove_wait_queue(&dev->w_wait, &wait); /*�Ӹ����ĵȴ�����ͷ�Ƴ�*/  

        set_current_state(TASK_RUNNING);  //���̴��ڿ�����״̬

        return ret;  

}  

   

   

/*ioctl �豸���ƺ���*/  

static long globalfifo_ioctl(struct file *filp,unsigned int cmd, unsigned long arg)

{  

    struct globalfifo_dev *dev = filp->private_data;/*����豸�ṹ��ָ��*/  

  

    switch(cmd){  

        case FIFO_CLEAR:  

            down(&dev->sem);                        /*����ź���*/  

            dev->current_len = 0;  

            memset(dev->mem, 0, GLOBALFIFO_SIZE);  

            up(&dev->sem);                          /*�ͷ��ź���*/  

  

            printk(KERN_INFO"globalfifo is set to zero\n");  

            break;  

  

        default:  

            return -EINVAL;  

    }  

    return 0;  

}  

   

/*�������е�������ѯ����*/  

static unsigned int globalfifo_poll(struct file *filp, poll_table *wait)  

{  

    unsigned int mask = 0;  

    struct globalfifo_dev *dev = filp->private_data;/*����豸�ṹ��ָ��*/  

  

    down(&dev->sem);  

    poll_wait(filp, &dev->r_wait, wait);  

    poll_wait(filp, &dev->w_wait, wait);  

  

    /*fifo�ǿ�*/  

    if(dev->current_len != 0){  

        mask |= POLLIN | POLLRDNORM; /*��ʾ���ݿ��Ի��*/  

    }  

  

    /*fifo ����*/  

    if(dev->current_len != GLOBALFIFO_SIZE){  

        mask |= POLLOUT | POLLWRNORM ; /*��ʾ���ݿ���д��*/  

    }  

  

    up(&dev->sem);  

    return mask; /*���������Ƿ�ɶ� ���д�� ״̬*/  

}  

  

/*�ļ��򿪺���*/  

int globalfifo_open(struct inode *inode, struct file *filp)  

{  

    /*���豸�ṹ����Ϊ�豸��˽����Ϣ*/  

    filp->private_data = globalfifo_devp;  

    return 0;  

}  

  

/*�ļ��ͷź���*/  

int globalfifo_release(struct inode *inode, struct file *filp)  

{  

    return 0;  

}  

  

/*�ļ������ṹ��*/  

static const struct file_operations globalfifo_fops = {  

    .owner = THIS_MODULE,  

    .read = globalfifo_read,  

    .write = globalfifo_write,  

    .unlocked_ioctl = globalfifo_ioctl,  

    .poll = globalfifo_poll,  

    .open = globalfifo_open,  

    .release = globalfifo_release,  

};  

  

/*��ʼ����ע��cdev*/  

static void globalfifo_setup_cdev(struct globalfifo_dev *dev, int index)  

{  

    int err, devno = MKDEV(globalfifo_major, index);  

  

    cdev_init(&dev->cdev, &globalfifo_fops);  

    dev->cdev.owner = THIS_MODULE;  

    err = cdev_add(&dev->cdev, devno, 1);  

    if(err)  

        printk(KERN_NOTICE "Error %d adding gloabalfifo %d", err, index);  

}  

   

/*�豸����ģ����غ���*/  

int globalfifo_init(void)  

{  

    int ret;  

    dev_t devno = MKDEV(globalfifo_major, 0);  

  

    /*�����豸��*/  

    if(globalfifo_major)  

        ret = register_chrdev_region(devno, 1, "globalfifo");  

    else{/*��̬�����豸��*/  

        ret = alloc_chrdev_region(&devno, 0, 1, "globalfifo");  

        globalfifo_major = MAJOR(devno);  

    }  

  

    if(ret < 0)  

        return ret;  

  

    /*��̬�����豸�ṹ����ڴ�*/  

    globalfifo_devp = kmalloc(sizeof(struct globalfifo_dev), GFP_KERNEL);  

    if(!globalfifo_devp){  

        ret = - ENOMEM;  

		goto fail_malloc;  

    }  

  

    memset(globalfifo_devp, 0, sizeof(struct globalfifo_dev));  

  

    globalfifo_setup_cdev(globalfifo_devp, 0);  

  

		sema_init(&globalfifo_devp->sem,1);             /*��ʼ���ź���*/  

    init_waitqueue_head(&globalfifo_devp->r_wait);  /*��ʼ�����ȴ�����ͷ*/  

    init_waitqueue_head(&globalfifo_devp->w_wait);  /*��ʼ��д�ȴ�����ͷ*/  

  

    return 0;  

  

fail_malloc: unregister_chrdev_region(devno, 1);  

             return ret;  

}  

   

void globalfifo_exit(void)  

{  

    cdev_del(&globalfifo_devp->cdev); /*ע��cdev*/  

    kfree(globalfifo_devp); /*�ͷ��豸�ṹ���ڴ�*/  

    unregister_chrdev_region(MKDEV(globalfifo_major, 0), 1); /*�ͷ��豸��*/  

}  

   

MODULE_AUTHOR("54geeker");  

MODULE_LICENSE("Dual BSD/GPL");    

module_init(globalfifo_init);  

module_exit(globalfifo_exit);  
