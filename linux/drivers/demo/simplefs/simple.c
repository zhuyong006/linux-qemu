/*
 * A Simple Filesystem for the Linux Kernel.
 *
 * Initial author: Sankar P <sankar.curiosity@gmail.com>
 * License: Creative Commons Zero License - http://creativecommons.org/publicdomain/zero/1.0/
 *
 * TODO: we need to split it into smaller files
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/buffer_head.h>
#include <linux/slab.h>
#include <linux/random.h>
#include <linux/version.h>
#include <linux/time64.h>

#include "super.h"

#define f_dentry f_path.dentry
/* A super block lock that must be used for any critical section operation on the sb,
 * such as: updating the free_blocks, inodes_count etc. */
static DEFINE_MUTEX(simplefs_sb_lock);
static DEFINE_MUTEX(simplefs_inodes_mgmt_lock);
#if 1
#define CDBG(fmt, args...) printk(fmt, ##args)
#else
#define CDBG(fmt, args...)
#endif
/* FIXME: This can be moved to an in-memory structure of the simplefs_inode.
 * Because of the global nature of this lock, we cannot create
 * new children (without locking) in two different dirs at a time.
 * They will get sequentially created. If we move the lock
 * to a directory-specific way (by moving it inside inode), the
 * insertion of two children in two different directories can be
 * done in parallel */
static DEFINE_MUTEX(simplefs_directory_children_update_lock);

static struct kmem_cache *sfs_inode_cachep;
static struct kmem_cache *sfs_entry_cachep;

/*����Ŀ¼��ÿһ�����Ϣ*/
struct simplefs_cache_entry {
	struct simplefs_dir_record record;
	struct list_head list;
	int entry_no;
};

/*��������Ŀ¼*/
struct simplefs_dir_cache {
	uint64_t dir_children_count;
	struct list_head used;
	struct list_head free;
};



static struct simplefs_dir_cache *simplefs_cache_alloc(void)
{
    struct simplefs_dir_cache *dir_cache;
    dir_cache = kzalloc(sizeof(struct simplefs_dir_cache), GFP_KERNEL);
    if (!dir_cache)
        return ERR_PTR(-ENOMEM);

    INIT_LIST_HEAD(&dir_cache->free);
    INIT_LIST_HEAD(&dir_cache->used);

    return dir_cache;
}

/*         ����˵��
    dir_cache:
    			  ��ǰĿ¼�Ļ���
    bh:
    			  �����е�ǰĿ¼��ŵ�ʵ����Ϣ
 */


static int dir_cache_build(struct simplefs_dir_cache *dir_cache, struct buffer_head *bh)
{
	struct simplefs_dir_record *record;
	struct simplefs_cache_entry *cache_entry;
	int i;

	record = (struct simplefs_dir_record *)bh->b_data;

	//SIMPLEFS_MAX_CHILDREN_CNT�����Ŀ¼������֧�ִ�����Inode����
	//��ΪĿǰ�����DATA_BLOCK�д�ŵĶ���simplefs_dir_record������������
	//�������ݿ��С/����Entry��ֵ
	for (i = 0; i < SIMPLEFS_DEFAULT_BLOCK_SIZE/sizeof(struct simplefs_dir_record); i++, record++) {
		//ΪĿ¼�е�ÿһ�����ݷ���һ������
		cache_entry = kmem_cache_alloc(sfs_entry_cachep, GFP_KERNEL);
		if (!cache_entry)
			return -ENOMEM;
		//������Ž�������
		cache_entry->entry_no = i;
		
		//Inode�ı���Ǵ�1��ʼ�ģ�������Ϊ0��˵����Ŀ¼�µ����Inode�Ѿ��ͷ�
		if (record->inode_no != 0) {
			//�����Ϊ0�����뵽Ŀ¼�Ļ����е�used������
			memcpy(&cache_entry->record, record, sizeof(struct simplefs_dir_record));
			list_add_tail(&cache_entry->list, &dir_cache->used);
		} else {
			//���Ϊ0����˵����Inode�Ѿ����ͷţ�����뵽Ŀ¼�����free������
			list_add_tail(&cache_entry->list, &dir_cache->free);
		}
	}

	return 0;
}

/*         ����˵��
    dir_cache:
    			  ��ǰĿ¼�Ļ���
    dentry:
    			  ��Ҫ���ҵ�entry��
 */
static struct simplefs_cache_entry *used_cache_entry_get(struct simplefs_dir_cache *dir_cache,struct dentry *dentry)
{
	struct simplefs_cache_entry *cache_entry;

	list_for_each_entry(cache_entry, &dir_cache->used, list) {
		if (!strcmp(cache_entry->record.filename, dentry->d_name.name)) {
			return cache_entry;
		}
	}

	return NULL;
}
/*         ����˵��
    dir_cache:
    			  ��ǰĿ¼�Ļ���
    ����ֵ:
    			  �ӵ�ǰĿ¼�����е�free������,���ص�һ�����е�Ԫ��
    			  
 */
static struct simplefs_cache_entry *free_cache_entry_get(struct simplefs_dir_cache *dir_cache)
{
	return list_first_entry(&dir_cache->free, struct simplefs_cache_entry, list);
}

/*         ����˵��
    head:
    			  Ŀ¼���������ͷ(used,free���п���)
    cache_entry:  ��������used/free����ͷ��entry��
    			  
    			  
 */

static void cache_entry_insert(struct list_head *head, struct simplefs_cache_entry *cache_entry)
{
	struct simplefs_cache_entry *tmp_entry;
	list_del(&cache_entry->list);

	list_for_each_entry(tmp_entry, head, list) {
		if (cache_entry->entry_no < tmp_entry->entry_no)
			break;
	}

	list_add_tail(&cache_entry->list, &tmp_entry->list);
}


//ͬ��������
void simplefs_sb_sync(struct super_block *vsb)
{
	struct buffer_head *bh;
	struct simplefs_super_block *sb = SIMPLEFS_SB(vsb)->sb;
	
	bh = sb_bread(vsb, SIMPLEFS_SUPERBLOCK_BLOCK_NUMBER);
	BUG_ON(!bh);

	bh->b_data = (char *)sb;
	/* ��ǻ������ײ�Ϊ�� */
	mark_buffer_dirty(bh);
	/* Ȼ��ͬ�� */
	sync_dirty_buffer(bh);
	/*�ͷ�bhָ��*/
	brelse(bh);
}

struct simplefs_inode *simplefs_inode_search(struct super_block *sb,
		struct simplefs_inode *start,
		struct simplefs_inode *search)
{
	uint64_t count = 0;
	//ÿ�����ݿ���4K��������ݿ�ȫ����ŵ���Inode�����Inode���������£�
	int icount = SIMPLEFS_DEFAULT_BLOCK_SIZE / sizeof(struct simplefs_inode);
	while (start->inode_no != search->inode_no && count < icount) {
		count++;
		start++;
	}

	if (start->inode_no == search->inode_no) {
		return start;
	}

	return NULL;
}
		
/*         ����˵��
    vsb:
    			  ������
    inode:
    			  �������뵽Inode Block Data��inode
    			  
 */

void simplefs_inode_add(struct super_block *vsb, struct simplefs_inode *inode)
{
	struct simplefs_sb_info *sb_info = SIMPLEFS_SB(vsb);
	struct buffer_head *bh;
	struct simplefs_inode *inode_iterator;

	if (mutex_lock_interruptible(&simplefs_inodes_mgmt_lock)) {
		sfs_trace("Failed to acquire mutex lock\n");
		return;
	}

	//���Inode��Ϣ��������
	bh = sb_bread(vsb, SIMPLEFS_INODESTORE_BLOCK_NUMBER);
	BUG_ON(!bh);
	//��Ϊ������������δ�ŵĶ���simplefs_inode�Ľṹ���������ǿ��ת��
	inode_iterator = (struct simplefs_inode *)bh->b_data;

	if (mutex_lock_interruptible(&simplefs_sb_lock)) {
		sfs_trace("Failed to acquire mutex lock\n");
		return;
	}

	/* Append the new inode in the end in the inode store */
	/*�Ƚ�inode_iteratorָ��inode_no��Ӧ�Ĵ洢��*/
	inode_iterator += inode->inode_no-1;
	//����Inode��Ϣ����Ӧ��λ��
	memcpy(inode_iterator, inode, sizeof(struct simplefs_inode));
	//���������е�Inode������������
	sb_info->sb->inodes_count++;
	//���������е�Inode bitmap�Ķ�Ӧλ��λ
	set_bit(inode->inode_no, &sb_info->imap);

	//�Ƚ���ǰ�����ݿ���Ϊ�࣬�ȴ���д����
	mark_buffer_dirty(bh);
	//ͬ������Ҳ��Ҫ����
	simplefs_sb_sync(vsb);
	/*�ͷ�Inode�����ݿ�*/
	brelse(bh);

	mutex_unlock(&simplefs_sb_lock);
	mutex_unlock(&simplefs_inodes_mgmt_lock);
}

/*         ����˵��
    vsb:
    			  ������
    inode:
    			  ����ɾ����Inode
    			  
 */

void simplefs_inode_del(struct super_block *vsb, struct simplefs_inode *inode)
{
	struct simplefs_sb_info *sb_info = SIMPLEFS_SB(vsb);
	struct buffer_head *bh;
	struct simplefs_inode *inode_iterator;

	if (mutex_lock_interruptible(&simplefs_inodes_mgmt_lock)) {
		sfs_trace("Failed to acquire mutex lock\n");
		return;
	}

	//���Inode��Ϣ��������
	bh = sb_bread(vsb, SIMPLEFS_INODESTORE_BLOCK_NUMBER);
	BUG_ON(!bh);
	//��Ϊ������������δ�ŵĶ���simplefs_inode�Ľṹ���������ǿ��ת��
	inode_iterator = (struct simplefs_inode *)bh->b_data;

	if (mutex_lock_interruptible(&simplefs_sb_lock)) {
		sfs_trace("Failed to acquire mutex lock\n");
		return;
	}

	/* Append the new inode in the end in the inode store */
	/*�Ƚ�inode_iteratorָ��inode_no��Ӧ�Ĵ洢��*/
	inode_iterator += inode->inode_no-1;
	//����Inode��Ϣ����Ӧ��λ��
	memset(inode_iterator, 0x0, sizeof(struct simplefs_inode));
	//���������е�Inode���������Լ�
	sb_info->sb->inodes_count--;
	//���������е�Inode bitmap�Ķ�Ӧλ��λ
	clear_bit(inode->inode_no, &sb_info->imap);

	//�Ƚ���ǰ�����ݿ���Ϊ�࣬�ȴ���д����
	mark_buffer_dirty(bh);
	//ͬ������Ҳ��Ҫ����
	simplefs_sb_sync(vsb);
	/*�ͷ�Inode�����ݿ�*/
	brelse(bh);

	mutex_unlock(&simplefs_sb_lock);
	mutex_unlock(&simplefs_inodes_mgmt_lock);
}

/* This function returns a blocknumber which is free.
 * The block will be removed from the freeblock list.
 *
 * In an ideal, production-ready filesystem, we will not be dealing with blocks,
 * and instead we will be using extents
 *
 * If for some reason, the file creation/deletion failed, the block number
 * will still be marked as non-free. You need fsck to fix this.*/
// sb->free_blocks��Ӧ��Bitλ���Ϊ1����ô˵����Ӧ�����ݿ���У���������ݿ�Busy

/*         ����˵��
    vsb:
    			  ������
    out:
    			  �������صĿ������ݿ������
    			  
 */
int simplefs_sb_get_a_freeblock(struct super_block *vsb, uint64_t * out)
{
	//ͨ���ں˱�׼��SuperBlock�ṹ��ȡ�ض��ļ�ϵͳ��SB�ṹ
	struct simplefs_super_block *sb = SIMPLEFS_SB(vsb)->sb;
	int i;
	int ret = 0;

	if (mutex_lock_interruptible(&simplefs_sb_lock)) {
		sfs_trace("Failed to acquire mutex lock\n");
		ret = -EINTR;
		goto end;
	}

	//��Ҫע����ǣ����ݿ��Ǵӵ�3����ʼ�ģ�ԭ���ǳ����飬inode��Ϣ�������ڵ��������������Ѿ����ļ�ϵͳ��
	// sb->free_blocks�е�ÿһ��Bit������һ�����ݿ�
	/* Loop until we find a free block. We start the loop from 3,
	 * as all prior blocks will always be in use */
	for (i = 3; i < SIMPLEFS_MAX_FILESYSTEM_OBJECTS_SUPPORTED; i++) {
		//���sb->free_blocks�Ķ�ӦBitΪ0������ζ�ŵ�ǰ���ݿ��ǿ��еĿ���ʹ��
		if (sb->free_blocks & (1 << i)) {
			break;
		}
	}

	//��������ѭ����3~SIMPLEFS_MAX_FILESYSTEM_OBJECTS_SUPPORTED,����һȦ����û���ҵ����е����ݿ�
	//��˵�����ļ�ϵͳû��ʣ��Ŀռ��ˣ����س�����Ϣ
	if (unlikely(i == SIMPLEFS_MAX_FILESYSTEM_OBJECTS_SUPPORTED)) {
		printk(KERN_ERR "No more free blocks available");
		ret = -ENOSPC;
		goto end;
	}

	//����ҵ����е����ݿ飬�򷵻ظ����ݿ������
	*out = i;

	//��Ȼ�ҵ��˿��е����ݿ飬��ô��Ҫ��sb->free_blocks�Ķ�ӦBit��λ
	/* Remove the identified block from the free list */
	sb->free_blocks &= ~(1 << i);

	//�������ǻ���Ҫ����Ӧ�����ݿ���Ϊdirty������д������
	simplefs_sb_sync(vsb);

end:
	mutex_unlock(&simplefs_sb_lock);
	return ret;
}

/*���ص�ǰ�ļ�ϵͳ�е�Inode����*/
static int simplefs_sb_get_objects_count(struct super_block *vsb,
					 uint64_t * out)
{
	struct simplefs_super_block *sb = SIMPLEFS_SB(vsb)->sb;

	if (mutex_lock_interruptible(&simplefs_inodes_mgmt_lock)) {
		sfs_trace("Failed to acquire mutex lock\n");
		return -EINTR;
	}
	*out = sb->inodes_count;
	mutex_unlock(&simplefs_inodes_mgmt_lock);

	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 11, 0)
/*���������"ls"ָ���ʱ��ᱻ���ȵ�*/
/*         ����˵��
    filp:
    			  ��ǰĿ¼���ļ�ָ��
    ctx:
    			  ��ʾ��������Ϣ

    ˵�����ӵ�ǰĿ¼�Ļ�����ȡ����ʹ�õ�entry��������ϱ���vfs
 */
static int simplefs_iterate(struct file *filp, struct dir_context *ctx)
#else
static int simplefs_readdir(struct file *filp, void *dirent, filldir_t filldir)
#endif
{
	loff_t pos;
	//ͨ��Ŀ¼���ļ�ָ�룬��ȡĿ¼�ṹ
	struct dentry *dentry = filp->f_path.dentry;
	//��ȡ��ǰĿ¼��Inode
	struct inode * parent_inode = dentry->d_inode;
	//Ŀ¼����ָ��
	struct simplefs_dir_cache *dir_cache = NULL;
	//��ʱ��������¼�����е�ÿһ���ļ�
	struct simplefs_cache_entry *cache_entry;
	//ͨ���ں˵�inodeָ���ȡ�ض��ļ�ϵͳ��inode����ָ��
	struct simplefs_inode *parent = SIMPLEFS_INODE(parent_inode);
	struct buffer_head *bh;

	CDBG("dentry inode no: %d\n",parent_inode->i_ino);
	
	dir_cache = (struct simplefs_dir_cache *)dentry->d_fsdata;

	//���Ŀ¼�Ļ��治���ڣ���Ϊ���Ŀ¼����һ���µĻ���
	if (!dir_cache) {
		CDBG("new simplefs_dir_cache\n");
		//Ϊ��ǰ��Ŀ¼����һ������
		dir_cache = simplefs_cache_alloc();
		if (IS_ERR(dentry->d_fsdata))
		    return -EINVAL;
		//�ӵ�ǰĿ¼�л�ȡ��Ϣ
		bh = sb_bread(parent_inode->i_sb, parent->data_block_number);
		BUG_ON(!bh);
		//Ϊ��ǰĿ¼������úͿ��������������
		dir_cache_build(dir_cache, bh);
		brelse(bh);
	}
	dentry->d_fsdata = dir_cache;

	pos = ctx->pos;
	
	if (pos) {
		/* FIXME: We use a hack of reading pos to figure if we have filled in all data.
		 * We should probably fix this to work in a cursor based model and
		 * use the tokens correctly to not fill too many data in each cursor based call */
		return 0;
	}

	list_for_each_entry(cache_entry, &dir_cache->used, list) {
		dir_emit(ctx, cache_entry->record.filename, SIMPLEFS_FILENAME_MAXLEN,
			cache_entry->record.inode_no, DT_UNKNOWN);		
		ctx->pos += sizeof(struct simplefs_dir_record);
		pos += sizeof(struct simplefs_dir_record);
	}


	return 0;
}

/* This functions returns a simplefs_inode with the given inode_no
 * from the inode store, if it exists. */
//��Inode����ʼ��ʼ����Inode�Ų�ѯ����Ӧ��Inode��Ϣ�������ء�
struct simplefs_inode *simplefs_get_inode(struct super_block *sb,
					  uint64_t inode_no)
{
	struct simplefs_inode *sfs_inode = NULL;
	struct simplefs_inode *inode_buffer = NULL;

	struct buffer_head *bh;

	/* The inode store can be read once and kept in memory permanently while mounting.
	 * But such a model will not be scalable in a filesystem with
	 * millions or billions of files (inodes) */
	//ָ��Inode����ʼ��
	bh = sb_bread(sb, SIMPLEFS_INODESTORE_BLOCK_NUMBER);
	BUG_ON(!bh);
	//����ǿ��ת��Ϊsimplefs_inode���͵�ָ��
	sfs_inode = (struct simplefs_inode *)bh->b_data;
	
	if (mutex_lock_interruptible(&simplefs_inodes_mgmt_lock)) {
		printk(KERN_ERR "Failed to acquire mutex lock %s +%d\n",
		       __FILE__, __LINE__);
		return NULL;
	}

	if(inode_no == 0)
	{
		printk("%s invalid inode_no\n",__func__);
	}
	
	//����һ�����е�Inode���棬���������е�Inode��Ϣ��������
	inode_buffer = kmem_cache_alloc(sfs_inode_cachep, GFP_KERNEL);
	//������Inode����Ϣ���ǰ���˳���ŵ�
	/*  
	 *  ���ڵ� ռ����1��Inode�������Inode�����ŷ�
	 */
	sfs_inode += inode_no-1;
	memcpy(inode_buffer, sfs_inode, sizeof(struct simplefs_inode));
	
	mutex_unlock(&simplefs_inodes_mgmt_lock);
	brelse(bh);
	return inode_buffer;
}

ssize_t simplefs_read(struct file * filp, char __user * buf, size_t len,
		      loff_t * ppos)
{
	/* After the commit dd37978c5 in the upstream linux kernel,
	 * we can use just filp->f_inode instead of the
	 * f->f_path.dentry->d_inode redirection */
	struct simplefs_inode *inode =
	    SIMPLEFS_INODE(filp->f_path.dentry->d_inode);
	struct buffer_head *bh;

	char *buffer;
	int nbytes;

	//���Ҫ�����ݵ�ƫ�Ƴ����˸�Inode�Ĵ�С����ôֱ�ӷ��ض�ȡ����Ϊ0
	if (*ppos >= inode->file_size) {
		/* Read request with offset beyond the filesize */
		return 0;
	}

	//�õ���Inode���������������ȡ����
	bh = sb_bread(filp->f_path.dentry->d_inode->i_sb,
					    inode->data_block_number);

	if (!bh) {
		printk(KERN_ERR "Reading the block number [%llu] failed.",
		       inode->data_block_number);
		return 0;
	}
	//��������ǿ��ת��ΪChar*
	buffer = (char *)bh->b_data;
	//��Ȼ�Ƕ�����Ҫ���ǵ��п������ȡ�ĳ��Ȼᳬ����Inode�Ĵ�С�������Ҫȡ�����е����ֵ
	nbytes = min((size_t) inode->file_size, len);
	//��Inode��ȡ�������ݴ��ݸ��û���
	if (copy_to_user(buf, buffer, nbytes)) {
		brelse(bh);
		printk(KERN_ERR
		       "Error copying file contents to the userspace buffer\n");
		return -EFAULT;
	}
	//���ڶ�ȡ��������ı���̵�������˲���Ҫͬ��������ֱ���ͷ����ݿ��ָ��
	brelse(bh);
	//�ı��α��ָ��
	*ppos += nbytes;
	//���ض�ȡ�ĳ���
	return nbytes;
}

/* Save the modified inode */
int simplefs_inode_save(struct super_block *sb, struct simplefs_inode *sfs_inode)
{
	struct simplefs_inode *inode_iterator;
	struct buffer_head *bh;
	//�ȶ�ȡInode��������
	bh = sb_bread(sb, SIMPLEFS_INODESTORE_BLOCK_NUMBER);
	BUG_ON(!bh);

	if (mutex_lock_interruptible(&simplefs_sb_lock)) {
		sfs_trace("Failed to acquire mutex lock\n");
		return -EINTR;
	}
	//��Inode����������ƥ��Ҫ���µ�Inode
	inode_iterator = simplefs_inode_search(sb,
		(struct simplefs_inode *)bh->b_data,
		sfs_inode);

	if (likely(inode_iterator)) {
		/*����Inode*/
		memcpy(inode_iterator, sfs_inode, sizeof(*inode_iterator));
		CDBG(KERN_INFO "The inode updated\n");
		//��Inode������������ΪDirty����ͬ��
		mark_buffer_dirty(bh);
		sync_dirty_buffer(bh);
	} else {
		mutex_unlock(&simplefs_sb_lock);
		printk(KERN_ERR
		       "The new filesize could not be stored to the inode.");
		return -EIO;
	}
	//�ͷŸ�������
	brelse(bh);

	mutex_unlock(&simplefs_sb_lock);

	return 0;
}

/* FIXME: The write support is rudimentary. I have not figured out a way to do writes
 * from particular offsets (even though I have written some untested code for this below) efficiently. */
ssize_t simplefs_write(struct file * filp, const char __user * buf, size_t len,
		       loff_t * ppos)
{
	/* After the commit dd37978c5 in the upstream linux kernel,
	 * we can use just filp->f_inode instead of the
	 * f->f_path.dentry->d_inode redirection */
	struct inode *inode;
	struct simplefs_inode *sfs_inode;
	struct buffer_head *bh;
	struct super_block *sb;

	char *buffer;

	int retval;

#if 0
	retval = generic_write_checks(filp, ppos, &len, 0);
	if (retval) {
		return retval;
	}
#endif
	//ͨ���ļ��õ��ں˶�Ӧ��Inodeָ��
	inode = filp->f_path.dentry->d_inode;
	//ͨ���ں˵�Inodeָ������õ��ض��ļ�ϵͳ��Inodeָ��
	sfs_inode = SIMPLEFS_INODE(inode);
	//ͨ��Inode�õ�SuperBlock
	sb = inode->i_sb;
	//��ȡ��Inodeָ������ݿ�
	bh = sb_bread(filp->f_path.dentry->d_inode->i_sb,
					    sfs_inode->data_block_number);

	if (!bh) {
		printk(KERN_ERR "Reading the block number [%llu] failed.",
		       sfs_inode->data_block_number);
		return 0;
	}
	
	//��������ǿ��ת��Ϊchar*��
	buffer = (char *)bh->b_data;

	/* Move the pointer until the required byte offset */
	//�ƶ���vfsָ����ƫ��λ��
	buffer += *ppos;

	//�����û��ռ�����ݵ���Ӧ�����ݿ���
	if (copy_from_user(buffer, buf, len)) {
		brelse(bh);
		printk(KERN_ERR
		       "Error copying file contents from the userspace buffer to the kernel space\n");
		return -EFAULT;
	}
	//֪ͨVFSָ��ƫ���˶���
	*ppos += len;
	//������������ΪDirty������д������
	mark_buffer_dirty(bh);
	sync_dirty_buffer(bh);
	//����ͷ����ݿ��ָ��
	brelse(bh);

	/* Set new size
	 * sfs_inode->file_size = max(sfs_inode->file_size, *ppos);
	 *
	 * FIXME: What to do if someone writes only some parts in between ?
	 * The above code will also fail in case a file is overwritten with
	 * a shorter buffer */
	if (mutex_lock_interruptible(&simplefs_inodes_mgmt_lock)) {
		sfs_trace("Failed to acquire mutex lock\n");
		return -EINTR;
	}
	//����Inode���ļ���С
	sfs_inode->file_size = *ppos;
	//��Ȼ������Inode����Ϣ����ôInode����Ϣ��ҲҪͬ��������
	retval = simplefs_inode_save(sb, sfs_inode);
	if (retval) {
		len = retval;
	}
	mutex_unlock(&simplefs_inodes_mgmt_lock);

	return len;
}

const struct file_operations simplefs_file_operations = {
	.read = simplefs_read,
	.write = simplefs_write,
};

const struct file_operations simplefs_dir_operations = {
	.owner = THIS_MODULE,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 11, 0)
	.iterate = simplefs_iterate,
#else
	.readdir = simplefs_readdir,
#endif
};

struct dentry *simplefs_lookup(struct inode *parent_inode,
			       struct dentry *child_dentry, unsigned int flags);

static int simplefs_create(struct inode *dir, struct dentry *dentry,
			   umode_t mode, bool excl);

static int simplefs_mkdir(struct inode *dir, struct dentry *dentry,
			  umode_t mode);
static int simplefs_unlink(struct inode *dir,struct dentry *dentry);

static struct inode_operations simplefs_inode_ops = {
	.create = simplefs_create,
	.lookup = simplefs_lookup,
	.mkdir = simplefs_mkdir,
	.unlink = simplefs_unlink,
	
};
/*
 *        		��������˵��
 *    dir��    			��ǰ����Ŀ¼��Inode
 *    dentry:  			dentry->d_name.name:���������ļ�����Ŀ¼��
 */
static int simplefs_create_fs_object(struct inode *dir, struct dentry *dentry,
				     umode_t mode)
{
	struct inode *inode;
	struct simplefs_inode *sfs_inode;
	struct simplefs_inode *parent_dir_inode;
	struct buffer_head *bh;
	struct simplefs_dir_record *dir_contents_datablock;
	uint64_t count;
	int ret;
	struct super_block *sb = dir->i_sb;
	struct dentry *parent_dentry = dentry->d_parent;
	struct simplefs_cache_entry *cache_entry;
	struct simplefs_dir_cache * dir_cache;
	struct simplefs_sb_info *sb_info = SIMPLEFS_SB(sb);


	BUG_ON(parent_dentry->d_inode != dir);

	dir_cache = (struct simplefs_dir_cache *)parent_dentry->d_fsdata;

	BUG_ON(!dir_cache);
	
	if (mutex_lock_interruptible(&simplefs_directory_children_update_lock)) {
		sfs_trace("Failed to acquire mutex lock\n");
		return -EINTR;
	}
	//ͨ�������Inode��ȡ������ļ�ϵͳ��SuperBlock
	sb = dir->i_sb;
	
	//��������˼�������������Ҫ����һ��Inode�ǲ���Ӧ�ÿ��¸��ļ�ϵͳ��Inodeλ���Ƿ�		
	//���п������������Ǵ����أ���ˣ�������Ҫ�õ���ǰ�ļ�ϵͳ�Ѿ�ʹ�õ�Inode������
	ret = simplefs_sb_get_objects_count(sb, &count);
	if (ret < 0) {
		mutex_unlock(&simplefs_directory_children_update_lock);
		return ret;
	}

	//���ж�Inode�����Ƿ��ˣ�����ǣ��򷵻��û�û�пռ䴴����
	if (unlikely(count >= SIMPLEFS_MAX_FILESYSTEM_OBJECTS_SUPPORTED)) {
		/* The above condition can be just == insted of the >= */
		printk(KERN_ERR
		       "Maximum number of objects supported by simplefs is already reached");
		mutex_unlock(&simplefs_directory_children_update_lock);
		return -ENOSPC;
	}

	//���ļ�ϵͳֻ֧��Ŀ¼����ͨ�ļ��Ĵ��������򷵻س���
	if (!S_ISDIR(mode) && !S_ISREG(mode)) {
		printk(KERN_ERR
		       "Creation request but for neither a file nor a directory");
		mutex_unlock(&simplefs_directory_children_update_lock);
		return -EINVAL;
	}
	
	//ͨ��SuperBlock����һ���յ�Inode  
	inode = new_inode(sb);
	if (!inode) {
		mutex_unlock(&simplefs_directory_children_update_lock);
		return -ENOMEM;
	}
	CDBG("sb imap inode no = %d\n",ffz(sb_info->imap));
	//�������Inodeָ���SuperBlock   
	inode->i_sb = sb;
	//�������Inode�Ĳ���ָ��
	inode->i_op = &simplefs_inode_ops;
	//�������Inode�Ĵ���ʱ��
	inode->i_atime = inode->i_mtime = inode->i_ctime = CURRENT_TIME;
	//�ӳ������inode map��ȡ����һ��Ϊ0��������(Bitλ)
	inode->i_ino = ffz(sb_info->imap);
	//�����ض��ļ�ϵͳ��Inode�ṹ
	sfs_inode = kmem_cache_alloc(sfs_inode_cachep, GFP_KERNEL);
	//�Ըýڵ��Inode�Ÿ�ֵ
	sfs_inode->inode_no = inode->i_ino;
	//���ں˱�׼�ڵ��˽��ָ��ָ��ǰ�ض��ļ�ϵͳ��Inode�ṹ
	inode->i_private = sfs_inode;
	//�����ļ�ϵͳ������
	sfs_inode->mode = mode;

	//���ļ�Ŀ¼�Լ���ͨ�ļ��ֱ������ã���Ҫע����ǣ������������һ��Ŀ¼����ô�������ʣ���ǰĿ¼��
	//��Inode�����϶�����Ϊ0��
	if (S_ISDIR(mode)) {
		CDBG(KERN_INFO "New directory creation request\n");
		sfs_inode->dir_children_count = 0;
		inode->i_fop = &simplefs_dir_operations;
	} else if (S_ISREG(mode)) {
		CDBG(KERN_INFO "New file creation request\n");
		sfs_inode->file_size = 0;
		//�����ͨ�ļ����ö�д����
		inode->i_fop = &simplefs_file_operations;
	}

	/* First get a free block and update the free map,
	 * Then add inode to the inode store and update the sb inodes_count,
	 * Then update the parent directory's inode with the new child.
	 *
	 * The above ordering helps us to maintain fs consistency
	 * even in most crashes
	 */
	//�ӳ������л�ȡ���е����ݿ�
	ret = simplefs_sb_get_a_freeblock(sb, &sfs_inode->data_block_number);
	if (ret < 0) {
		printk(KERN_ERR "simplefs could not get a freeblock");
		mutex_unlock(&simplefs_directory_children_update_lock);
		return ret;
	}
	//�½�һ��Inode��Ҫ����Inode������������ͬ��
	simplefs_inode_add(sb, sfs_inode);

	/*�ӵ�ǰĿ¼�����е�free������ȡ��һ�����е�entry��*/
	cache_entry = free_cache_entry_get(dir_cache);

	/*���˸���Inode�������������ǻ���Ҫ��һ����:�ڸ�Ŀ¼(Inode)���棬��Ӹ�Inode����Ϣ*/
	//��ȻҪ�����Ϣ����������ȡ�õ�ǰ��Ŀ¼�Ľṹ��Ϣ��ͨ���ں˵ı�׼Inode��ȡ�ض��ļ�ϵͳ��Inode
	//��Ϣ
	parent_dir_inode = SIMPLEFS_INODE(dir);
	//ͨ��simplefs_inode�еĳ�Ա�Ӷ���ȡ��������Ϣ
	bh = sb_bread(sb, parent_dir_inode->data_block_number);
	BUG_ON(!bh);
	//��Ҫ֪������Ŀ¼Inode�д�ŵ����ݽṹ���ǹ̶��ģ��������ǿ��ת��
	dir_contents_datablock = (struct simplefs_dir_record *)bh->b_data;

	/* Navigate to the last record in the directory contents */
	/*��Ҫע����Ǹ�Ŀ¼�����е�entryҲ�ǰ���˳���ŵģ�cache_entry->entry_noָ���
      ���Ǵ�ŵ���ţ����Ǵ�free������ȡ����entry�ǽ�����used�����
	*/
	dir_contents_datablock += cache_entry->entry_no;
	/*�����е�Ŀ¼entry��Ϣ����*/
	dir_contents_datablock->inode_no = sfs_inode->inode_no;
	strcpy(dir_contents_datablock->filename, dentry->d_name.name);
	/*����������Ŀ¼�����ŵ����ݵ�entry�Ļ�����*/
	memcpy(&cache_entry->record, dir_contents_datablock,sizeof(struct simplefs_dir_record));
	/*����ǰ��cache_entry���뵽used��������*/
	cache_entry_insert(&dir_cache->used, cache_entry);

	/*����Ŀ¼ָ������ݿ�����Ϊdirty���������д�����̣�֮���ͷ�*/
	mark_buffer_dirty(bh);
	sync_dirty_buffer(bh);
	brelse(bh);

	if (mutex_lock_interruptible(&simplefs_inodes_mgmt_lock)) {
		mutex_unlock(&simplefs_directory_children_update_lock);
		sfs_trace("Failed to acquire mutex lock\n");
		return -EINTR;
	}
	//����Ŀ¼�е�dir_children_countҲ����
	parent_dir_inode->dir_children_count++;
	//ͬ�����Ǹ�����Inode�������������������ȻҲҪͬ������
	ret = simplefs_inode_save(sb, parent_dir_inode);
	if (ret) {
		mutex_unlock(&simplefs_inodes_mgmt_lock);
		mutex_unlock(&simplefs_directory_children_update_lock);

		/* TODO: Remove the newly created inode from the disk and in-memory inode store
		 * and also update the superblock, freemaps etc. to reflect the same.
		 * Basically, Undo all actions done during this create call */
		return ret;
	}

	mutex_unlock(&simplefs_inodes_mgmt_lock);
	mutex_unlock(&simplefs_directory_children_update_lock);
	//����ǰInode���丸Ŀ¼����
	inode_init_owner(inode, dir, mode);
	//����ǰinode�󶨵�dentry��
	d_add(dentry, inode);

	return 0;
}

/*
 *        		��������˵��
 *    dir��    			��ǰ����Ŀ¼��Inode
 *    dentry:  			dentry->d_name.name:��ɾ�����ļ�
 */
static int simplefs_unlink(struct inode *dir,struct dentry *dentry)
{
	struct inode *inode;
	struct simplefs_inode *sfs_inode;
	struct simplefs_inode *parent_dir_inode;
	struct buffer_head *bh;
	struct simplefs_dir_record *dir_contents_datablock;
	uint64_t count;
	int ret;
	struct super_block *sb = dir->i_sb;
	struct dentry *parent_dentry = dentry->d_parent;
	struct simplefs_cache_entry *cache_entry;
	struct simplefs_dir_cache * dir_cache;
	struct simplefs_sb_info *sb_info = SIMPLEFS_SB(sb);

	/*��ȡ��ɾ���ļ���Ӧ���ļ�ϵͳ��inode����*/
	sfs_inode = SIMPLEFS_INODE(dentry->d_inode);

	dir_cache = (struct simplefs_dir_cache *)parent_dentry->d_fsdata;

	/*��Ŀ¼�ж�Ӧ�������*/
	parent_dir_inode = SIMPLEFS_INODE(dir);
	//ͨ��simplefs_inode�еĳ�Ա�Ӷ���ȡ��������Ϣ
	bh = sb_bread(sb, parent_dir_inode->data_block_number);
	dir_contents_datablock = (struct simplefs_dir_record *)bh->b_data;
	cache_entry = used_cache_entry_get(dir_cache,dentry);
	dir_contents_datablock += cache_entry->entry_no;
	memset(dir_contents_datablock,0x0,sizeof(struct simplefs_dir_record));
	/*����Ŀ¼ָ������ݿ�����Ϊdirty���������д�����̣�֮���ͷ�*/
	mark_buffer_dirty(bh);
	sync_dirty_buffer(bh);
	brelse(bh);

	/*�ӵ�ǰĿ¼��used�����У�ɾ����ǰ��cache_entry*/
	list_del(&cache_entry->list);
	//���ͷŵ�cache_entry���뵽��ǰĿ¼��free������
	list_add(&cache_entry->list,&dir_cache->free);
	//��Ŀ¼�µ�inode����һ
	dir_cache->dir_children_count--;
	//����Ŀ¼�е�dir_children_countҲ�Լ�
	parent_dir_inode->dir_children_count--;
	//ͬ�����Ǹ�����Inode�������������������ȻҲҪͬ������
	ret = simplefs_inode_save(sb, parent_dir_inode);

	//��Inode�Ĵ洢���У������Ӧ��Inode
	simplefs_inode_del(sb,sfs_inode);

	//�ͷ��ں˵�inode�ṹ
	__destroy_inode(dentry->d_inode);

	dput(dentry);

	return 0;
}

static int simplefs_mkdir(struct inode *dir, struct dentry *dentry,
			  umode_t mode)
{
	/* I believe this is a bug in the kernel, for some reason, the mkdir callback
	 * does not get the S_IFDIR flag set. Even ext2 sets is explicitly */
	 
	CDBG("%s LINE = %d\n",__func__,__LINE__);
	return simplefs_create_fs_object(dir, dentry, S_IFDIR | mode);
}

static int simplefs_create(struct inode *dir, struct dentry *dentry,
			   umode_t mode, bool excl)
{
	CDBG("%s LINE = %d\n",__func__,__LINE__);

	return simplefs_create_fs_object(dir, dentry, mode);
}
/*         ����˵��
    parent_inode:
    			  ��ǰĿ¼��Inode
    child_dentry:
    			   ��ǰ��ѯ��Inode,child_dentry->d_name.name���Ի�ȡ����
    flags:
    
 */
struct dentry *simplefs_lookup(struct inode *parent_inode,
			       struct dentry *child_dentry, unsigned int flags)
{
	struct super_block *sb = parent_inode->i_sb;
	struct dentry *parent_dentry;
	struct simplefs_dir_cache *dir_cache;
	struct simplefs_cache_entry *cache_entry;
	struct inode *inode;
	struct simplefs_inode *sfs_inode;

	CDBG("%s LINE = %d,%s\n",__func__,__LINE__,
		child_dentry->d_name.name);
	
	//�õ���Ŀ¼
	parent_dentry = child_dentry->d_parent;
	
	if (parent_dentry->d_inode != parent_inode)
		return ERR_PTR(-ENOENT);
	
	//�õ���Ŀ¼��˽������: Ŀ¼Cache
	dir_cache = (struct simplefs_dir_cache *)parent_dentry->d_fsdata;
	
	CDBG("%s LINE = %d\n",__func__,__LINE__);

	//��Ŀ¼cache�е�used�������ҵ�����ǰ��ѯ�ļ���cache_entry
	cache_entry = used_cache_entry_get(dir_cache, child_dentry);

	//���cache_entryΪ�գ�˵�����ļ�����Ŀ¼�У���Ҫcreat
	if (!cache_entry)
		goto out;
	
	CDBG("%s check inode_no = %d\n",__func__,cache_entry->record.inode_no);

	
	sfs_inode = simplefs_get_inode(sb, cache_entry->record.inode_no);
	if (!sfs_inode)
		return ERR_PTR(-ENOENT);

	
	inode = new_inode(sb);
	inode->i_ino = cache_entry->record.inode_no;
	inode_init_owner(inode, parent_inode, sfs_inode->mode);
	inode->i_sb = sb;
	inode->i_op = &simplefs_inode_ops;

	if (S_ISDIR(inode->i_mode))
		inode->i_fop = &simplefs_dir_operations;
	else if (S_ISREG(inode->i_mode))
		inode->i_fop = &simplefs_file_operations;
	else
		printk(KERN_ERR
		       "Unknown inode type. Neither a directory nor a file");

	/* FIXME: We should store these times to disk and retrieve them */
	inode->i_atime = inode->i_mtime = inode->i_ctime = CURRENT_TIME;

	inode->i_private = sfs_inode;

	d_add(child_dentry, inode);
	
	CDBG(KERN_ERR
	       "No inode found for the filename [%s]\n",
	       child_dentry->d_name.name);

out:
	return NULL;

}


/**
 * Simplest
 */
void simplefs_destory_inode(struct inode *inode)
{
	struct simplefs_inode *sfs_inode = SIMPLEFS_INODE(inode);

	printk(KERN_INFO "Freeing private data of inode %p (%lu)\n",
	       sfs_inode, inode->i_ino);
	kmem_cache_free(sfs_inode_cachep, sfs_inode);
}

static const struct super_operations simplefs_sops = {
	.destroy_inode = simplefs_destory_inode,
};

static void simplefs_dentry_release(struct dentry *dentry)
{
	struct simplefs_dir_cache *dir_cache = dentry->d_fsdata;
	struct simplefs_cache_entry *tmp, *cache_entry;

	if (dir_cache) {
		list_for_each_entry_safe(cache_entry, tmp, &dir_cache->free, list) {
		list_del(&cache_entry->list);
		kmem_cache_free(sfs_entry_cachep, cache_entry);
		}

		list_for_each_entry_safe(cache_entry, tmp, &dir_cache->used, list) {
		list_del(&cache_entry->list);
		kmem_cache_free(sfs_entry_cachep, cache_entry);
		}
	}

	kfree(dir_cache);
	dentry->d_fsdata = NULL;
}

static const struct dentry_operations simplefs_dentry_operations = {
	.d_release = simplefs_dentry_release,
};

static void fill_imap(struct super_block *sb)
{
	int i;
	struct simplefs_sb_info *sb_info = sb->s_fs_info;
	struct simplefs_inode *simple_inode;
	struct buffer_head *bh;
	int icount = SIMPLEFS_DEFAULT_BLOCK_SIZE / sizeof(struct simplefs_inode);

	bh = sb_bread(sb, SIMPLEFS_INODESTORE_BLOCK_NUMBER);
	simple_inode = (struct simplefs_inode *)bh->b_data;
	
	CDBG("%s start,root inode = %d\n",__func__,simple_inode->inode_no);
	CDBG("sb imap inode no = %d\n",ffz(sb_info->imap));

	/*��1��bitԤ�����ã���Ϊroot�ڵ������Ǵ�1��ʼ��*/
	for (i = 0; i < SIMPLEFS_START_INO; i++)
		set_bit(i, &sb_info->imap);

	/*��inode��Ԫ�������У�����ȶԣ����Ѿ�ʹ�õ�inode��bitmap�б��*/
	for (i = SIMPLEFS_START_INO; i < icount; i++) {
		if (simple_inode->inode_no != 0) {
			printk("func %s, line %d, ino %lld\n", __func__, __LINE__, simple_inode->inode_no);
			set_bit(simple_inode->inode_no, &sb_info->imap);
		}
		simple_inode++;
	}
	
	CDBG("sb imap inode no = %d\n",ffz(sb_info->imap));
	CDBG("%s end\n",__func__);

	brelse(bh);
}


/* This function, as the name implies, Makes the super_block valid and
 * fills filesystem specific information in the super block */
int simplefs_fill_super(struct super_block *sb, void *data, int silent)
{
	struct inode *root_inode;
	struct buffer_head *bh;
	struct simplefs_super_block *sb_disk;
	struct simplefs_sb_info *sb_info;
	int ret = -EPERM;

	sb_info = kzalloc(sizeof(struct simplefs_sb_info),GFP_KERNEL);
	bh = sb_bread(sb, SIMPLEFS_SUPERBLOCK_BLOCK_NUMBER);
	BUG_ON(!bh);
	//��ȡ�����д�ŵ�super block����ʵ����
	sb_disk = (struct simplefs_super_block *)bh->b_data;

	//���ó����黺��ָ����ں�sb��ָ��
	sb_info->sb = sb_disk;
	//���ó����黺��ָ���buffer_head
	sb_info->bh = bh;

	printk(KERN_INFO "The magic number obtained in disk is: [%llu]\n",
	       sb_disk->magic);

	if (unlikely(sb_disk->magic != SIMPLEFS_MAGIC)) {
		printk(KERN_ERR
		       "The filesystem that you try to mount is not of type simplefs. Magicnumber mismatch.");
		goto release;
	}

	if (unlikely(sb_disk->block_size != SIMPLEFS_DEFAULT_BLOCK_SIZE)) {
		printk(KERN_ERR
		       "simplefs seem to be formatted using a non-standard block size.");
		goto release;
	}

	printk(KERN_INFO
	       "simplefs filesystem of version [%llu] formatted with a block size of [%llu] detected in the device.\n",
	       sb_disk->version, sb_disk->block_size);

	/* A magic number that uniquely identifies our filesystem type */
	//sb�е�ħ���ʹ����е�һ��
	sb->s_magic = SIMPLEFS_MAGIC;

	/* For all practical purposes, we will be using this s_fs_info as the super block */
	//ʹ���ں˵�sb˽��ָ��ָ�򳬼���Ļ���
	sb->s_fs_info = sb_info;
	//������ǰ�ļ�ϵͳ������ݿ�Ϊ4K
	sb->s_maxbytes = SIMPLEFS_DEFAULT_BLOCK_SIZE;
	//ʵ��Inode��destroyָ�룬���ļ�ϵͳ���ļ���ɾ�������Ӧ��Inode����ᱻ��
	//����ָ��ĺ����ͷ�
	sb->s_op = &simplefs_sops;
	
	sb->s_d_op = &simplefs_dentry_operations;
	/*���³����黺���д�ŵ�inode bitmap*/
	fill_imap(sb);

	//Ϊ���ǵĸ��ڵ����һ��Inode
	root_inode = new_inode(sb);
	//���ø��ڵ��Inode���
	root_inode->i_ino = SIMPLEFS_ROOTDIR_INODE_NUMBER;
	//�������Inode��һ��Ŀ¼����Ϊ�Ǹ�dentry������������Ŀ¼ΪNULL
	inode_init_owner(root_inode, NULL, S_IFDIR);
	//ָ�������ļ�ϵͳ�ĳ�����
	root_inode->i_sb = sb;
	//��Ϊ��Ҫ�ڸ��ڵ��½��в����������Ҫʵ�ֽڵ�Ĳ���ָ��
	/*
	 * 	1.����һ����ͨ�ļ�������createָ��;
	 * 	2.����һ����ͨ�ļ�֮ǰ������Ҫ�ȵ���lookupָ��;
	 * 	3.����һ��Ŀ¼������mkdir;
	 */
	root_inode->i_op = &simplefs_inode_ops;
	//Provide Io Operation For UserSpace,eg:readdir
	//�Ƿ���Ҫ֧���ļ��Ĳ����������д,mmap��
	root_inode->i_fop = &simplefs_dir_operations;
	//Inode�Ĵ���ʱ���
	root_inode->i_atime = root_inode->i_mtime = root_inode->i_ctime =
	    CURRENT_TIME;
	//��Ȼ�Ǹ��ڵ㣬��Ȼ����Ҫ��ȡ���ڵ�����ݣ������i_private����ָ����ڵ���Ϣ��
	/*
		��Ϣ����:
		1.���ڵ��
		2.���ڵ����������ݴ�ŵ�λ��-> DATA_BLOCK_BUMBER
		3.���ڵ�������ӽڵ����
	*/
	root_inode->i_private =
	    simplefs_get_inode(sb, SIMPLEFS_ROOTDIR_INODE_NUMBER);

	
	/* TODO: move such stuff into separate header. */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0)
	//Super Block��Ҫ��֪��ǰ�ļ�ϵͳ�ĸ�dentry�������dentry������Ҳ��������һ��inode
	sb->s_root = d_make_root(root_inode);
#else
	sb->s_root = d_alloc_root(root_inode);
	if (!sb->s_root)
		iput(root_inode);
#endif

	if (!sb->s_root) {
		ret = -ENOMEM;
		goto release;
	}


	ret = 0;
release:
	brelse(bh);

	return ret;
}

static struct dentry *simplefs_mount(struct file_system_type *fs_type,
				     int flags, const char *dev_name,
				     void *data)
{
	struct dentry *ret;

	ret = mount_bdev(fs_type, flags, dev_name, data, simplefs_fill_super);

	if (unlikely(IS_ERR(ret)))
		printk(KERN_ERR "Error mounting simplefs");
	else
		printk(KERN_INFO "simplefs is succesfully mounted on [%s]\n",
		       dev_name);

	return ret;
}

static void simplefs_kill_superblock(struct super_block *sb)
{
	struct simplefs_sb_info *sb_info = sb->s_fs_info;

	printk(KERN_INFO
	       "simplefs superblock is destroyed. Unmount succesful.\n");
	/* This is just a dummy function as of now. As our filesystem gets matured,
	 * we will do more meaningful operations here */

	kill_block_super(sb);
	//brelse(sb_info->bh);
	kfree(sb_info);
	return;
}

struct file_system_type simplefs_fs_type = {
	.owner = THIS_MODULE,
	.name = "simplefs",
	.mount = simplefs_mount,
	.kill_sb = simplefs_kill_superblock,
	.fs_flags = FS_REQUIRES_DEV,
};

static int simplefs_init(void)
{
	int ret;

	printk("%s LINE = %d\n",__func__,__LINE__);
	sfs_inode_cachep = kmem_cache_create("sfs_inode_cache",
	                                     sizeof(struct simplefs_inode),
	                                     0,
	                                     (SLAB_RECLAIM_ACCOUNT| SLAB_MEM_SPREAD),
	                                     NULL);
	
	sfs_entry_cachep = kmem_cache_create("sfs_entry_cachep",
										sizeof(struct simplefs_cache_entry),
										0,
										(SLAB_RECLAIM_ACCOUNT| SLAB_MEM_SPREAD),
										NULL);

	if (!sfs_inode_cachep) {
		return -ENOMEM;
	}
	
	if (!sfs_entry_cachep) {
		return -ENOMEM;
	}

	ret = register_filesystem(&simplefs_fs_type);
	if (likely(ret == 0))
		printk(KERN_INFO "Sucessfully registered simplefs\n");
	else
		printk(KERN_ERR "Failed to register simplefs. Error:[%d]", ret);

	return ret;
}

static void simplefs_exit(void)
{
	int ret;

	ret = unregister_filesystem(&simplefs_fs_type);
	kmem_cache_destroy(sfs_inode_cachep);

	if (likely(ret == 0))
		printk(KERN_INFO "Sucessfully unregistered simplefs\n");
	else
		printk(KERN_ERR "Failed to unregister simplefs. Error:[%d]",
		       ret);
}

module_init(simplefs_init);
module_exit(simplefs_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sankar P");
