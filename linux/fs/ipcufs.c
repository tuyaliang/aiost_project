/*
 * ipcufs remote filesystem proxy for Socionext
 *
 * Copyright (C) 2015 Linaro, Ltd
 * Andy Green <andy.green@linaro.org>
 *
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/mount.h>
#include <linux/pagemap.h>
#include <linux/uaccess.h>
#include <linux/of_platform.h>
#include <linux/mailbox_client.h>
#include <linux/dma-mapping.h>
#include <linux/buffer_head.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/statfs.h>
#include <linux/aio.h>
#include <linux/writeback.h>
#include <linux/workqueue.h>
#include <linux/fsnotify.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/uio.h>

#include <linux/backing-dev.h>


#include "ipcufs.h"
#include <uapi/linux/shared_mem.h>

#define CREATE_TRACE_POINTS
#include <trace/events/ipcufs.h>

#if 0
#define IPCUFS_USE_FAT
#endif

#define IPCUFS_END_POS 0x7fffffff

static struct inode *ipcufs_iget(struct super_block *sb, int pos,
				 const char *path, struct dentry *dentry);
static struct inode *ipcufs_iget_register(struct inode *inode, struct super_block *sb, int pos,
					  const char *path, struct dentry *dentry);
static const struct inode_operations ipcufs_dir_inode_operations;
static const struct inode_operations ipcufs_inode_operations;

static u32 inode_num;

struct fake_inode {
	char *name;
	u64 i_target;		/* first inode inside directory or 0 */
	u64 i_sibling;		/* next inode in same directory or 0 */
	int rw;
	unsigned int fNo;
	loff_t pos;
};

static const struct fake_inode init_fake_inode[] = {
	[1] = {
	       .name = "/",
	       .i_target = 0,
	       .i_sibling = 0,
	       },
};

static struct backing_dev_info ipcufs_bdi[IPCUFS_MAX_DRV_NUM];

#define ARRAY_SIZE_IPCUFS_OPEN_FILE	(20)

static struct fake_inode **fake_file;
/* open file multiple management structure */
static struct ipcufs_open_mng_struct {
	struct fake_inode *open_file[ARRAY_SIZE_IPCUFS_OPEN_FILE];
	int older_flg[ARRAY_SIZE_IPCUFS_OPEN_FILE];
} ipcufs_open_mng;

static unsigned int mount_cnt;

#ifdef IPCUFS_USE_FAT
#define fnamecmp(s1, s2)	strcasecmp(s1, s2)
#else
#define fnamecmp(s1, s2)	strcmp(s1, s2)
#endif

static int ipcufs_find_unused_inode(u32 s)
{
	int n = 1;

	while (n < IPCUFS_MAX_INODE_NUM) {

		if (!fake_file[s][n].name)
			return n;
		n++;
	}

	/* there are no free inodes */

	return 0;
}

static void ipcufs_alloc_drive(u32 s)
{
	int n;

	if (fake_file[s]) {
		pr_warn("%s:%d [WARN] redundant fake-file allocation.\n",
			__func__, __LINE__);
		return;
	}

	fake_file[s] =
	    kzalloc(sizeof(struct fake_inode) * inode_num,
		    GFP_NOWAIT);

	for (n = 1; n < ARRAY_SIZE(init_fake_inode); n++) {
		fake_file[s][n] = init_fake_inode[n];
		if (fake_file[s][n].name)
			fake_file[s][n].name = kstrdup(init_fake_inode[n].name,
						       GFP_KERNEL);
	}

	if (mount_cnt == 0) {
		for (n = 0; n < ARRAY_SIZE_IPCUFS_OPEN_FILE; n++) {
			ipcufs_open_mng.open_file[n] = NULL;
			ipcufs_open_mng.older_flg[n] = 0;
		}
	}

	mount_cnt++;
}

static void ipcufs_free_drive(u32 s)
{
	int n;

	/* fake_file[s][0] is reserved */
	for (n = 1; n < inode_num; n++)
		kfree(fake_file[s][n].name);	/* it is safe to pass NULL */

	kfree(fake_file[s]);
	fake_file[s] = NULL;

	mount_cnt--;
}

static int ipcufs_get_fake_ino(u32 s, int parent, const char *filename)
{
	u64 *pp = &fake_file[s][parent].i_target;

	while (*pp) {
		if (!fnamecmp(filename, fake_file[s][*pp].name))
			return *pp;
		pp = &fake_file[s][*pp].i_sibling;
	}

	*pp = ipcufs_find_unused_inode(s);
	fake_file[s][*pp].name = kstrdup(filename, GFP_NOWAIT);
	fake_file[s][*pp].i_target = 0;
	fake_file[s][*pp].i_sibling = 0;

	return *pp;
}

/* Count free fake inode entries */
static u32 ipcufs_count_free_ino(u32 s)
{
	u32 n, count = 0;

	/* Note: n = 0 is resereved for rootdir inode which must be there */
	for (n = 1; n < IPCUFS_MAX_INODE_NUM; n++)
		if (!fake_file[s][n].name)
			count++;

	return count;
}

#ifdef IPCUFS_USE_FAT
static char *ipcufs_get_real_filename(u32 s, int parent, const char *filename)
{

	u64 *pp = &fake_file[s][parent].i_target;

	while (*pp) {
		if (!fnamecmp(filename, fake_file[s][*pp].name))
			return fake_file[s][*pp].name;
		pp = &fake_file[s][*pp].i_sibling;
	}

	return NULL;
}
#endif

static int ipcufs_delete_fake_ino(u32 s, int parent, int ino)
{
	u64 *pp;

	/* walk the parent dir siblings until we find the link */
	pp = &fake_file[s][parent].i_target;

	while (*pp && *pp != ino)
		pp = &fake_file[s][*pp].i_sibling;

	if (!*pp)
		return 1;

	/* snip him out of linked-list */
	*pp = fake_file[s][ino].i_sibling;

	/* free up his name and mark not in use */
	kfree(fake_file[s][ino].name);
	fake_file[s][ino].name = NULL;

	return 0;
}

static int ipcufs_rename_fake_ino(u32 s1, u32 s2, int parent, int new_parent,
				  int ino, const char *name)
{
	u64 *pp;

	/* remove from src directory */
	pp = &fake_file[s1][parent].i_target;

	while (*pp && *pp != ino)
		pp = &fake_file[s1][*pp].i_sibling;

	if (!*pp)
		return 1;

	*pp = fake_file[s1][ino].i_sibling;

	/* attach to dst directory */
	pp = &fake_file[s2][new_parent].i_target;

	while (*pp)
		pp = &fake_file[s2][*pp].i_sibling;

	if (*pp)
		return 1;

	*pp = ino;
	fake_file[s2][ino].i_sibling = 0;

	/* change name field */
	kfree(fake_file[s1][ino].name);
	fake_file[s2][ino].name = kstrdup(name, GFP_NOWAIT);

	return 0;
}

static struct mutex mlock;
static struct mutex flock;
static void *shm;
static dma_addr_t shm_paddr;
static void *page_shm;
static dma_addr_t page_shm_paddr;
static void *dio_buf;
static T_IPCU_IF mssg;
static struct workqueue_struct *ipcufs_wq;
static struct work_struct work;
static u32 ipcu_unit, send_ch, recv_ch;
static struct sni_ipcu_device send_dev, recv_dev;

static int ipcufs_ipcu_init(void)
{
	int rc, i;

	for (i = 0; i < IPCUFS_MAX_DRV_NUM; i++) {
		if (fake_file[i])
			return 0;
	}

	rc = sni_ipcu_ch_init(ipcu_unit,
			      send_ch, SNI_IPCU_DIR_SEND, (void *)&send_dev);
	if (rc < 0) {
		pr_err("%s:%d [ERROR] cannot init ipcu channel.\n", __func__,
		       __LINE__);
		goto err4;
	}

	rc = sni_ipcu_opench(ipcu_unit, send_ch, SNI_IPCU_DIR_SEND);
	if (rc < 0) {
		pr_err("%s:%d [ERROR] sni_ipcu_opench(): %d\n", __func__,
		       __LINE__, rc);
		goto err3;
	}

	rc = sni_ipcu_ch_init(ipcu_unit,
			      recv_ch, SNI_IPCU_DIR_RECV, (void *)&recv_dev);

	if (rc < 0) {
		pr_err("%s:%d [ERROR] cannot init ipcu channel.\n", __func__,
		       __LINE__);
		goto err2;
	}

	rc = sni_ipcu_opench(ipcu_unit, recv_ch, SNI_IPCU_DIR_RECV);
	if (rc < 0) {
		pr_err("%s:%d [ERROR] sni_ipcu_opench(): %d\n", __func__,
		       __LINE__, rc);
		goto err1;
	}

	return 0;

 err1:
	sni_ipcu_ch_exit(ipcu_unit, recv_ch, (void *)&recv_dev);
 err2:
	sni_ipcu_closech(ipcu_unit, send_ch, SNI_IPCU_DIR_SEND);
 err3:
	sni_ipcu_ch_exit(ipcu_unit, send_ch, (void *)&send_dev);

 err4:
	return -EINVAL;
}

static int ipcufs_ipcu_uninit(void)
{
	int rc, i;

	for (i = 0; i < IPCUFS_MAX_DRV_NUM; i++) {
		if (fake_file[i])
			return 0;
	}

	sni_ipcu_ch_exit(ipcu_unit, send_ch, (void *)&send_dev);

	rc = sni_ipcu_closech(ipcu_unit, send_ch, SNI_IPCU_DIR_SEND);
	if (rc < 0) {
		pr_err("%s:%d [ERROR] sni_ipcu_closech(): %d\n", __func__,
		       __LINE__, rc);
	}

	sni_ipcu_ch_exit(ipcu_unit, recv_ch, (void *)&recv_dev);

	rc = sni_ipcu_closech(ipcu_unit, recv_ch, SNI_IPCU_DIR_RECV);
	if (rc < 0) {
		pr_err("%s:%d [ERROR] sni_ipcu_closech(): %d\n", __func__,
		       __LINE__, rc);
	}

	return 0;
}

static void ipcufs_work_handler(struct work_struct *work)
{
	int ret = 0;
	int recv_buf[9];

	ret = sni_ipcu_recv_msg_kernel(ipcu_unit,
				       recv_ch,
				       &recv_buf, sizeof(mssg), FLAG_RECV_WAIT);

	if (ret < 0) {
		pr_err("%s:%d [ERROR] sni_ipcu_recv_msg_kernel(): %d\n",
		       __func__, __LINE__, ret);
	}

}

static int ipcufs_transfer(void *p, int len)
{
	int ret = 0;
	int i, tmp;

	trace_ipcufs_transfer_enter(p, len);
	mutex_lock(&mlock);

	if (len <= 4096)
		memcpy_toio(shm, p, len);
	else {
		tmp = len;
		for(i = 0; tmp > 4096; i++) {
			memcpy_toio((char*)(shm + (4096 * i)),
				    (char*)(p + (4096 * i)), 4096);
			tmp = tmp - 4096;
		}
		memcpy_toio((char*)(shm + (4096 * i)),
			    (char*)(p + (4096 * i)), tmp);
	}

	mssg.id = send_ch;
	mssg.buf = shm_paddr;
	mssg.len = len;
	mssg.cont = 0;

	wmb();

	queue_work(ipcufs_wq, &work);

	ret = sni_ipcu_send_msg_kernel(ipcu_unit,
				       send_ch,
				       &mssg, sizeof(mssg), FLAG_SEND_NOTIFY);

	if (ret < 0) {
		pr_err("%s:%d [ERROR] sni_ipcu_send_msg_kernel(): %d\n",
		       __func__, __LINE__, ret);
		goto end;
	}

	flush_workqueue(ipcufs_wq);

	if (len <= 4096)
		memcpy_fromio(p, shm, len);
	else {
		tmp = len;
		for(i = 0; tmp > 4096; i++) {
			memcpy_fromio(((char*)p + (4096 * i)),
				      ((char*)shm + (4096 * i)), 4096);
			tmp = tmp - 4096;
		}
		memcpy_fromio(((char*)p + (4096 * i)),
			      ((char*)shm + (4096 * i)), tmp);
	}

	rmb();

	ret = sni_ipcu_ack_send(ipcu_unit, recv_ch);
	if (ret < 0) {
		pr_err("%s:%d [ERROR] sni_ipcu_send_msg(): %d\n", __func__,
		       __LINE__, ret);
		goto end;
	}

 end:

	mutex_unlock(&mlock);
	trace_ipcufs_transfer_exit(p, len, ret);

	return ret;
}

static int ipcufs_file_opened(struct fake_inode *ff)
{
	if (!ff)
		return 0;

	return ff->fNo > 0 ? 1 : 0;
}

static int ipcufs_close(struct fake_inode *ff);

static int ipcufs_open(struct fake_inode *ff, u32 drive, const char *path,
		       int rw)
{
	struct ipcufs_info_table *it =
	    kzalloc(sizeof(struct ipcufs_info_table), GFP_NOWAIT);
	int ret = 0;
	int n;
	int set_done = 0;

	it->u.action = IFSA_OPEN;
	it->c.open.drive = drive;
	strncpy(it->c.open.path, path, IPCUFS_MAX_PATH_LEN);
	it->c.open.mode = (u32) rw;

	if (ipcufs_transfer((void *)it, sizeof(struct ipcufs_info_table))) {
		ret = -EIO;
		goto end;
	}

	if (it->c.open.err) {
		ret = -EIO;
		goto end;
	}

	ff->rw = rw;
	ff->fNo = it->c.open.fNo;

	/* search vacant manager */
	for (n = 0; n < ARRAY_SIZE_IPCUFS_OPEN_FILE; n++) {
		if (set_done == 0 && ipcufs_open_mng.open_file[n] == NULL) {
			ipcufs_open_mng.open_file[n] = ff;
			ipcufs_open_mng.older_flg[n] = 1;
			set_done = 1;
		} else {
			if (ipcufs_open_mng.older_flg[n] > 0)
				ipcufs_open_mng.older_flg[n]++;
		}
	}

	/* search older file */
	for (n = 0; n < ARRAY_SIZE_IPCUFS_OPEN_FILE; n++) {
		if (ipcufs_open_mng.older_flg[n] > ARRAY_SIZE_IPCUFS_OPEN_FILE) {
			/* close older file */
			ipcufs_close(ipcufs_open_mng.open_file[n]);

			wmb();

			if (set_done == 0) {
				/* add vacant manager */
				ipcufs_open_mng.open_file[n] = ff;
				ipcufs_open_mng.older_flg[n] = 1;
				set_done = 1;
			}
		}
	}

 end:
	kfree(it);
	return ret;
}

static int ipcufs_close(struct fake_inode *ff)
{
	int n;

	struct ipcufs_info_table *it =
	    kzalloc(sizeof(struct ipcufs_info_table), GFP_NOWAIT);
	int ret = 0;

	if (!ipcufs_file_opened(ff))
		goto end;

	it->u.action = IFSA_CLOSE;
	it->c.close.fNo = ff->fNo;
	ff->rw = 0;
	ff->fNo = 0;
	ff->pos = 0;

	if (ipcufs_transfer((void *)it, sizeof(struct ipcufs_info_table))) {
		ret = -EIO;
		goto end;
	}

	if (it->c.close.err) {
		ret = -EIO;
		goto end;
	}

	for (n = 0; n < ARRAY_SIZE_IPCUFS_OPEN_FILE; n++) {
		if (ipcufs_open_mng.open_file[n] == ff) {
			ipcufs_open_mng.open_file[n] = NULL;
			ipcufs_open_mng.older_flg[n] = 0;
			goto end;
		}
	}

 end:
	kfree(it);
	return ret;
}

static int ipcufs_file_validate(struct fake_inode *ff, u32 drive, char *path,
				int rw)
{
	int n;

	if (ipcufs_file_opened(ff) && ff->rw == rw)
		return 0;

	for (n = 0; n < ARRAY_SIZE_IPCUFS_OPEN_FILE; n++) {
		if (ipcufs_open_mng.open_file[n] == ff) {
			if (ipcufs_close(ff))
				return -1;
		}
	}

	if (ipcufs_open(ff, drive, path, rw))
		return -1;

	return 0;
}

static int ipcufs_file_invalidate(struct fake_inode *ff)
{
	return ipcufs_close(ff);
}

static int ipcufs_file_overwrite(struct file *file)
{
	struct ipcufs_info_table *it =
	    kzalloc(sizeof(struct ipcufs_info_table), GFP_NOWAIT);
	struct dentry *dentry = file->f_path.dentry;
	char *buf = kzalloc(IPCUFS_MAX_PATH_LEN, GFP_NOWAIT);
	char *path_in = dentry_path_raw(dentry, buf, IPCUFS_MAX_PATH_LEN);

	struct inode *inode = file->f_inode;
	u32 drive = (u32) inode->i_sb->s_fs_info;
	struct fake_inode *ff = &fake_file[drive][inode->i_ino];

	if (ipcufs_file_invalidate(ff))
		goto err;

	it->u.action = IFSA_IDELETE;
	it->c.idel.drive = (u32) dentry->d_sb->s_fs_info;
	strncpy(it->c.idel.path, path_in, IPCUFS_MAX_PATH_LEN);

	if (ipcufs_transfer((void *)it, sizeof(struct ipcufs_info_table)))
		goto err;

	if (it->c.idel.err)
		goto err;

	memset(it, 0, sizeof(struct ipcufs_info_table));

	it->u.action = IFSA_ICREATE;
	it->c.icreate.drive = (u32) dentry->d_sb->s_fs_info;
	strncpy(it->c.icreate.path, path_in, IPCUFS_MAX_PATH_LEN);

	if (ipcufs_transfer((void *)it, sizeof(struct ipcufs_info_table)))
		goto err;

	if (it->c.icreate.err)
		goto err;

	kfree(it);
	kfree(buf);

	return 0;
 err:
	kfree(it);
	kfree(buf);

	return -1;
}

static int __ipcufs_iget_transfer(struct inode *i, u32 drv, const char *path,
				  struct dentry *dentry);

/*
 * and the rest of this is generic for Linux and shouldn't need changing --->
 */

static struct dentry *ipcufs_lookup(struct inode *dir, struct dentry *dentry,
				    unsigned int flags)
{
	struct inode *inode = NULL;
	struct inode *tmp_inode = kzalloc(sizeof(struct inode), GFP_NOWAIT);
	char *buf = kzalloc(IPCUFS_MAX_PATH_LEN, GFP_NOWAIT);
	char *path_in = dentry_path_raw(dentry, buf, IPCUFS_MAX_PATH_LEN);
	int ino;
	int ret;

#ifdef IPCUFS_USE_FAT
	char *realname;

	realname = ipcufs_get_real_filename((u32) dentry->d_sb->s_fs_info,
					    dir->i_ino, dentry->d_name.name);
	if (realname) {
		if (strcmp(realname, dentry->d_name.name))
			goto end;
	}
#endif

	mutex_lock(&flock);

	tmp_inode->i_state |= I_NEW;
	ret = __ipcufs_iget_transfer(tmp_inode, (u32)dentry->d_sb->s_fs_info,
				     path_in, dentry);
	if (ret < 0)
		goto end;

	ino = ipcufs_get_fake_ino((u32) dentry->d_sb->s_fs_info,
				  dir->i_ino, dentry->d_name.name);

	if (!ino)
		goto end;

	inode = ipcufs_iget_register(tmp_inode, dir->i_sb, ino, path_in,
				     dentry);
 end:

	d_add(dentry, inode);

	mutex_unlock(&flock);

	kfree(buf);
	kfree(tmp_inode);

	return 0;
}

/* A cache for failure entry */
static struct readdir_cache {
	u32 drv;
	int parent;
	loff_t pos;
	char name[IPCUFS_MAX_NAME_LEN];
	int name_len;
	int ino;
	int dtype;
} readdir_fail_cache;
/*
 * Note: if RTOS side ensures that each drive/each inode uses different
 * file-pointer for readdir (support concurrent readdir in drive level
 * or directory level), linux also can prepare readdir cache in such
 * level of context. For example if RTOS has different readdir context
 * for drive, we can allocate readdir_fail_cache[drv] and keep it.
 */

static void clear_readdir_cache(struct readdir_cache *cache)
{
	cache->parent = 0;
	cache->pos = 0;
}

static bool dir_emit_cache(struct dir_context *ctx, struct readdir_cache *cache)
{
	bool ret = dir_emit(ctx, cache->name, cache->name_len, cache->ino,
			    cache->dtype);
	if (ret)
		clear_readdir_cache(cache);

	return ret;
}

static void save_readdir_cache(struct readdir_cache *cache, u32 drv,
	int parent, loff_t pos, char *name, int name_len, int ino, int dtype)
{
	cache->drv = drv;
	cache->parent = parent;
	cache->pos = pos;
	strncpy(cache->name, name, name_len);
	cache->name_len = name_len;
	cache->ino = ino;
	cache->dtype = dtype;
}

/* IPCUFS set information for readdir */
static int ipcufs_set_readdir_info(struct dir_context *ctx, u32 drv, int parent,
				   char* name, u32 type)
{
	int ret, dtype;
	u32 name_len;
	char* realname;

	if (type)
		dtype = DT_DIR;
	else
		dtype = DT_REG;

	ret = ipcufs_get_fake_ino(drv, parent,
		(const char *)&name);
	name_len = strlen(name);
#ifdef IPCUFS_USE_FAT
	realname = ipcufs_get_real_filename(drv, parent, name);
#else
	realname = name;
#endif
	if (!dir_emit(ctx, realname, name_len, ret, dtype)) {
		pr_debug("dir_emit fails\n");
		save_readdir_cache(&readdir_fail_cache, drv, parent,
			ctx->pos, realname, name_len, ret, dtype);
		return 0;
	}
	ctx->pos++;
	return ret;
}

/* IPCUFS uses ctx->pos as index number of the directory entries. */
static int ipcufs_readdir(struct file *file, struct dir_context *ctx)
{
	struct inode *i = file_inode(file);
	struct ipcufs_info_table_readdir *it =
	    kzalloc(sizeof(struct ipcufs_info_table_readdir), GFP_NOWAIT);
	char *buf = kzalloc(IPCUFS_MAX_PATH_LEN, GFP_NOWAIT);
	u32 drv = (u32) i->i_sb->s_fs_info;
	int p = i->i_ino;
	int parent = p;
	char *path_in =
	    dentry_path_raw(file->f_path.dentry, buf, IPCUFS_MAX_PATH_LEN);
	u16 open = 1;
	int j;
	int ret = 0;

	pr_debug("%s: ctx->pos=%lld, p=%d\n", __func__, (u64) ctx->pos, p);

	mutex_lock(&flock);

	if (ctx->pos == IPCUFS_END_POS)
		goto end;

	/* Note that '.' (pos == 0) and '..' (pos == 1) are fixed */
	if (ctx->pos < 2) {
		if (!dir_emit_dots(file, ctx))
			goto end;
	} else if (ctx->pos > 2) {
		/* Processing cache */
		if (ctx->pos == readdir_fail_cache.pos &&
		    parent == readdir_fail_cache.parent &&
		    drv == readdir_fail_cache.drv) {
			if (!dir_emit_cache(ctx, &readdir_fail_cache)) {
				pr_debug("%s: failed to dir_emit_cache.", __func__);
				goto end;
			}
			ctx->pos++;
		} else
			clear_readdir_cache(&readdir_fail_cache);

		/* ctx->pos starts from 2 but open >> 1 starts from 1 */
		open = (ctx->pos - 1) << 1;
	} else {/* pos == 2 */
		/*
		 * Here, we need to open the dir and enum dirent again from
		 * the beginning.
		 */
		open = 1;
	}

	do {
		it->u.action = IFSA_READDIR;
		it->c.rdir.drive = drv;
		strncpy(it->c.rdir.path, path_in, IPCUFS_MAX_PATH_LEN);
		it->c.rdir.open = open;
		memset(&it->c.rdir.filename, 0, IPCUFS_MAX_NAME_LEN);
		if (open == 0)
			it->c.rdir.request_nums = IPCUFS_READDIR_MAX;
		else
			it->c.rdir.request_nums = 0;
		it->c.rdir.response_nums = 0;

		for (j = 0; j < IPCUFS_READDIR_MAX; j++) {
			memset(&it->c.rdir.filenames[j], 0, IPCUFS_MAX_NAME_LEN);
			memset(&it->c.rdir.types[j], 0, sizeof(u32));
		}

		open = 0;

		if (ipcufs_transfer
		    ((void *)it, sizeof(struct ipcufs_info_table_readdir)))
			break;

		/* read single */
		if (it->c.rdir.err)
			break;

		p = ipcufs_set_readdir_info(ctx, drv, parent,
					    it->c.rdir.filename, it->c.rdir.type);
		if (!p) {
			ret = -EINVAL;
			goto end;
		}

		/* read addition */
		for (j = 0; j < it->c.rdir.response_nums; j++) {
			if (it->c.rdir.errors[j]) {
				ctx->pos = IPCUFS_END_POS;
				goto end;
			}

			p = ipcufs_set_readdir_info(ctx, drv, parent,
						it->c.rdir.filenames[j], it->c.rdir.types[j]);
			if (!p) {
				ret = -EINVAL;
				goto end;
			}
		}
	} while (p);

	ctx->pos = IPCUFS_END_POS;

 end:
	mutex_unlock(&flock);
	kfree(it);
	kfree(buf);

	return ret;
}

static const struct file_operations ipcufs_dir_operations = {
	.read = generic_read_dir,
	.iterate = ipcufs_readdir,
	.llseek = default_llseek,
};

static loff_t _ipcufs_fillsize(struct inode *inode, struct page *page)
{
	loff_t offset, size;

	offset = page_offset(page);
	size = i_size_read(inode);

	if (offset > size) {
		pr_err("offset > size\n");
		return 0;
	}

	size -= offset;
	return size > PAGE_SIZE ? PAGE_SIZE : size;
}

static void _ipcufs_clean_page(struct page *page, int ret)
{

	if (!ret)
		SetPageUptodate(page);
	else
		SetPageError(page);

	flush_dcache_page(page);
	unlock_page(page);
}

static int ipcufs_readpage(struct file *file, struct page *page)
{
	struct inode *inode = page->mapping->host;
	u32 drive = (u32) inode->i_sb->s_fs_info;
	struct fake_inode *ff = &fake_file[drive][inode->i_ino];
	struct ipcufs_info_table *it =
	    kzalloc(sizeof(struct ipcufs_info_table), GFP_NOWAIT);
	char *buf = kzalloc(IPCUFS_MAX_PATH_LEN, GFP_NOWAIT);

	loff_t offset, size;
	unsigned long fillsize;
	void *readbuf;
	int ret;
	char *path_in =
	    dentry_path_raw(file->f_path.dentry, buf, IPCUFS_MAX_PATH_LEN);

	mutex_lock(&flock);

	trace_ipcufs_readpage_enter(inode->i_ino, page);

	if (ipcufs_file_validate(ff, drive, path_in, IPCUFS_OPEN_READ))
		goto end;

	readbuf = kmap(page);

	if (!readbuf) {
		ret = -ENOMEM;
		goto end;
	}

	offset = page_offset(page);
	size = i_size_read(inode);
	fillsize = 0;
	ret = 0;
	if (offset < size) {
		size -= offset;
		fillsize = size > PAGE_SIZE ? PAGE_SIZE : size;

		it->u.action = IFSA_READPAGE;
		it->c.pages.fNo = (u32) ff->fNo;
		it->c.pages.pa[0] = page_shm_paddr;
		it->c.pages.len[0] = fillsize;
		it->c.pages.offset = offset;

		if (ipcufs_transfer
		    ((void *)it, sizeof(struct ipcufs_info_table))) {
			ret = -EIO;
			goto end;
		}

		if (it->c.page.err == IPCUFS_ERR_NG) {
			ret = -EIO;
			goto end;
		}
	}

	memcpy_fromio(readbuf, page_shm, fillsize);
	if (fillsize < PAGE_SIZE)
		memset(readbuf + fillsize, 0, PAGE_SIZE - fillsize);

	ff->pos = offset + fillsize;

	if (!ret)
		SetPageUptodate(page);
	else
		SetPageError(page);

 out:
	flush_dcache_page(page);
	kunmap(page);
	unlock_page(page);
	kfree(it);
	kfree(buf);

	trace_ipcufs_readpage_exit(inode->i_ino, ret);
	mutex_unlock(&flock);

	return ret;

 end:
	ipcufs_close(ff);
	goto out;
}

static int _ipcufs_readpages_transfer(struct ipcufs_info_table *it,
				      struct page **pages, int count_pages)
{
	int ret = 0, n;

	if (ipcufs_transfer((void *)it, sizeof(struct ipcufs_info_table)) ||
	    it->c.page.err == IPCUFS_ERR_NG)
		ret = -EIO;

	for (n = 0; n < count_pages; n++)
		_ipcufs_clean_page(pages[n], ret);

	return ret;
}

struct filler_args {
	struct ipcufs_info_table *it;
	struct page *pages[MAX_IPCUFS_PAGES_AT_ONCE];
	int count_pages;
	int count_blocks;
};

static int _ipcufs_rp_filler(void *v, struct page *page)
{
	struct filler_args *a = v;
	struct inode *inode = page->mapping->host;
	struct ipcufs_info_table *it = a->it;
	int n = a->count_pages;
	int nb = a->count_blocks;
	int ret;
	unsigned long paddr, plen;

	if (n != 0 &&
	    (a->pages[n - 1]->index + 1 != page->index)) {
		/* find a gap in the list. read right before it. */
		ret = _ipcufs_readpages_transfer(it, a->pages, n);
		if (ret)
			return ret;

		/* reset ipcufs info table for reusing */
		a->count_pages = a->count_blocks = n = nb = 0;
		memset(it->c.pages.pa, 0, sizeof(u32) * 64);
		memset(it->c.pages.len, 0, sizeof(u32) * 64);
	}

	paddr = __pfn_to_phys(page_to_pfn(page));
	plen = _ipcufs_fillsize(inode, page);

	if (n == 0) {
		it->c.pages.offset = page_offset(page);
		it->c.pages.pa[nb] = paddr;
		it->c.pages.len[nb] = plen;
	} else if (it->c.pages.pa[nb] + it->c.pages.len[nb] == paddr) {
		/* This page is physical-contiguous, just add it */
		it->c.pages.len[nb] += plen;
	} else {
		/* This is discontiguous page, proceed block number */
		nb++;
		it->c.pages.pa[nb] = paddr;
		it->c.pages.len[nb] = plen;
		a->count_blocks++;
	}

	flush_dcache_page(page);
	dma_unmap_page(NULL, paddr, plen, DMA_FROM_DEVICE);

	a->pages[n] = page;
	a->count_pages++;

	return 0;
}

static int ipcufs_readpages(struct file *file, struct address_space *mapping,
			    struct list_head *pages, unsigned nr_pages)
{
	struct filler_args a;
	struct inode *i = file_inode(file);
	u32 drive = (u32) i->i_sb->s_fs_info;
	struct fake_inode *ff = &fake_file[drive][i->i_ino];
	struct ipcufs_info_table *it =
	    kzalloc(sizeof(struct ipcufs_info_table), GFP_NOWAIT);
	char *buf = kzalloc(IPCUFS_MAX_PATH_LEN, GFP_NOWAIT);
	char *path_in =
	    dentry_path_raw(file->f_path.dentry, buf, IPCUFS_MAX_PATH_LEN);
	int ret;
	int n;

	mutex_lock(&flock);

	trace_ipcufs_readpages_enter(i->i_ino, nr_pages);

	if (nr_pages > MAX_IPCUFS_PAGES_AT_ONCE) {
		ret = -EINVAL;
		goto end;
	}

	if (ipcufs_file_validate(ff, drive, path_in, IPCUFS_OPEN_READ))
		goto end;

	it->u.action = IFSA_READPAGE;
	it->c.pages.fNo = (u32) ff->fNo;

	a.it = it;
	a.count_pages = 0;
	a.count_blocks = 0;

	ret = read_cache_pages(mapping, pages, _ipcufs_rp_filler, &a);
	if (ret) {
		pr_err("read_cache_pages returned %d\n", ret);
		goto end;
	} else if (!a.count_pages) {
		ret = 0;
		goto end;
	}

	ret = _ipcufs_readpages_transfer(it, a.pages, a.count_pages);
	if (ret)
		goto end;

	ff->pos = it->c.pages.offset;
	for (n = 0; n < a.count_blocks; n++)
		ff->pos += a.it->c.pages.len[n];

 out:
	mutex_unlock(&flock);
	kfree(it);
	kfree(buf);
	trace_ipcufs_readpages_exit(i->i_ino, ret);

	return ret;

 end:
	ipcufs_close(ff);
	goto out;
}

/*
 * Some parts mimic simple_write_end()@fs/libfs.c, since we use
 * simple_write_begin() for ops->write_begin.
 */
static int ipcufs_write_end(struct file *file, struct address_space *mapping,
			    loff_t pos, unsigned len, unsigned copied,
			    struct page *page, void *fsdata)
{
	struct inode *i = page->mapping->host;
	u32 drive = (u32) i->i_sb->s_fs_info;
	struct fake_inode *ff = &fake_file[drive][i->i_ino];

	struct ipcufs_info_table *it =
	    kzalloc(sizeof(struct ipcufs_info_table), GFP_NOWAIT);
	char *buf = kzalloc(IPCUFS_MAX_PATH_LEN, GFP_NOWAIT);
	char *path_in =
	    dentry_path_raw(file->f_path.dentry, buf, IPCUFS_MAX_PATH_LEN);
	int ret = 0;
	unsigned offset;	/* Offset inside given page */

	mutex_lock(&flock);

	/* Overwrite operation sanity check */
	if (pos == 0 && ipcufs_file_overwrite(file)) {
		ret = -EIO;
		goto err;
	}
	if (ipcufs_file_validate(ff, drive, path_in, IPCUFS_OPEN_WRITE))
		goto err;

	offset = pos & (PAGE_CACHE_SIZE - 1);

	/* zero the stale part of the page if we did a short copy */
	if (copied < len) {
		zero_user(page, offset + copied, len - copied);

		len = copied;	/* we don't bother to copy stale part */
	}

	memcpy(page_shm, kmap(page) + offset, len);

	it->u.action = IFSA_WRITEPAGE;
	it->c.pages.fNo = (u32) ff->fNo;
	it->c.pages.pa[0] = page_shm_paddr;
	it->c.pages.len[0] = len;
	it->c.pages.offset = pos;

	if (ipcufs_transfer((void *)it, sizeof(struct ipcufs_info_table))) {
		ret = -EIO;
		goto err;
	}

	if (it->c.pages.err) {
		ret = -EIO;
		goto err;
	}

	ff->pos = pos + len;
	if (ff->pos > i->i_size)
		i_size_write(i, ff->pos);

	unlock_page(page);
	page_cache_release(page);
	ret = len;
 out:
	kunmap(page);
	kfree(it);
	kfree(buf);
	mutex_unlock(&flock);
	trace_ipcufs_write_end(i->i_ino, ret);

	return ret;

 err:
	ipcufs_close(ff);
	goto out;
}

static u32 ipcufs_dio_read(int fNo, void *addr, u32 size, loff_t offset)
{
	struct ipcufs_info_table *it =
	    kzalloc(sizeof(struct ipcufs_info_table), GFP_NOWAIT);
	void *buf;

	if (virt_addr_valid(addr)) {
		buf = addr;
	} else if (access_ok(VERIFY_WRITE, addr, size)) {
		buf = dio_buf;
	} else {
		pr_err("%s: invalid addr\n", __func__);
		goto err;
	}

	it->u.action = IFSA_READPAGE;
	it->c.pages.fNo = (u32) fNo;
	it->c.pages.pa[0] = virt_to_phys(buf);
	it->c.pages.len[0] = size;
	it->c.pages.offset = offset;

	dma_map_single(NULL, buf, size, DMA_FROM_DEVICE);

	if (ipcufs_transfer((void *)it, sizeof(struct ipcufs_info_table)))
		goto err;

	dma_unmap_single(NULL, it->c.pages.pa[0], size, DMA_FROM_DEVICE);

	if (!virt_addr_valid(addr)) {
		if (copy_to_user(addr, buf, size))
			goto err;
	}

	kfree(it);

	return it->c.pages.err;

 err:
	kfree(it);

	return IPCUFS_ERR_NG;
}

static u32 ipcufs_dio_write(int fNo, void *addr, u32 size, loff_t offset)
{

	struct ipcufs_info_table *it =
	    kzalloc(sizeof(struct ipcufs_info_table), GFP_NOWAIT);
	void *buf;

	if (virt_addr_valid(addr)) {
		buf = addr;
	} else if (access_ok(VERIFY_WRITE, addr, size)) {
		buf = dio_buf;
		if (copy_from_user(buf, addr, size))
			goto err;
	} else {
		pr_err("%s: invalid addr\n", __func__);
		goto err;
	}

	it->u.action = IFSA_WRITEPAGE;
	it->c.pages.fNo = (u32) fNo;
	it->c.pages.pa[0] = virt_to_phys(buf);
	it->c.pages.len[0] = size;
	it->c.pages.offset = offset;

	dma_map_single(NULL, buf, size, DMA_TO_DEVICE);

	if (ipcufs_transfer((void *)it, sizeof(struct ipcufs_info_table)))
		goto err;

	dma_unmap_single(NULL, it->c.pages.pa[0], size, DMA_TO_DEVICE);

	kfree(it);

	return it->c.pages.err;

 err:
	kfree(it);

	return IPCUFS_ERR_NG;
}

static ssize_t ipcufs_direct_IO(struct kiocb *iocb, struct iov_iter *iter,
				loff_t pos)
{
	struct file *file = iocb->ki_filp;
	struct inode *inode = file->f_inode;
	u32 drive = (u32) inode->i_sb->s_fs_info;
	struct fake_inode *ff = &fake_file[drive][inode->i_ino];
	int rw = iov_iter_rw(iter);
	void *addr = iter->iov->iov_base;
	u32 size = size = iov_iter_count(iter);
	char *buf = kzalloc(IPCUFS_MAX_PATH_LEN, GFP_NOWAIT);
	char *path_in =
	    dentry_path_raw(file->f_path.dentry, buf, IPCUFS_MAX_PATH_LEN);

	u32 err;

	trace_ipcufs_direct_io_enter(inode->i_ino, size);
	mutex_lock(&flock);

	if (rw == READ) {
		if (ipcufs_file_validate(ff, drive, path_in, rw))
			goto err;

		err = ipcufs_dio_read(ff->fNo, addr, size, pos);

	} else if (rw == WRITE) {
		if (pos == 0 && ipcufs_file_overwrite(file))
			goto err;

		if (ipcufs_file_validate(ff, drive, path_in, rw))
			goto err;

		err = ipcufs_dio_write(ff->fNo, addr, size, pos);
	} else
		goto err;

	if (err == IPCUFS_ERR_NG)
		goto err;

	iocb->ki_pos += size;
	ff->pos = pos + size;

	if (err == IPCUFS_ERR_EOF)
		ipcufs_close(ff);
 out:
	kfree(buf);
	mutex_unlock(&flock);
	trace_ipcufs_direct_io_exit(inode->i_ino, size);

	return size;

 err:
	ipcufs_close(ff);
	size = 0;	/* TODO: return correct error */
	goto out;
}

static int ipcufs_flush(struct file *file, fl_owner_t id)
{
	struct inode *inode = file_inode(file);
	u32 drive = (u32) inode->i_sb->s_fs_info;
	struct fake_inode *ff = &fake_file[drive][inode->i_ino];

	mutex_lock(&flock);

	if (ipcufs_close(ff)) {
		mutex_unlock(&flock);
		return -EIO;
	}

	mutex_unlock(&flock);

	return 0;
}

static int ipcufs_update_inode(struct inode *, struct dentry *);

static int ipcufs_file_open(struct inode *inode, struct file *file)
{
	/* Update inode and discard cache if needed */
	return ipcufs_update_inode(inode, file->f_path.dentry);
}

static int ipcufs_file_close(struct inode *inode, struct file *filp)
{
	u32 drive = (u32) inode->i_sb->s_fs_info;
	struct fake_inode *ff = &fake_file[drive][inode->i_ino];

	mutex_lock(&flock);

	ipcufs_close(ff);

	mutex_unlock(&flock);

	return 0;
}

static const struct address_space_operations ipcufs_aops = {
	.readpage = ipcufs_readpage,
	.readpages = ipcufs_readpages,
	.write_begin = simple_write_begin,
	.write_end = ipcufs_write_end,
	.direct_IO = ipcufs_direct_IO,
};

const struct file_operations ipcufs_file_operations = {
	.llseek = generic_file_llseek,
	.read_iter = generic_file_read_iter,
	.mmap = generic_file_readonly_mmap,
	.open = ipcufs_file_open,
	.release = ipcufs_file_close,
	.fsync = __generic_file_fsync,
	.splice_read = generic_file_splice_read,
	.write_iter = generic_file_write_iter,
	.flush = ipcufs_flush,
};

static void __ipcufs_init_ops(struct inode *i, struct dentry *dentry)
{
	if ((i->i_mode & S_IFMT) == S_IFDIR) {
		i->i_op = &ipcufs_dir_inode_operations;
		i->i_fop = &ipcufs_dir_operations;
		/* directory inodes start off with i_nlink == 2 (for "." entry) */
		inc_nlink(i);
		/* '..' increases the nlink of parent directory */
		if (dentry)
			inc_nlink(dentry->d_parent->d_inode);
	} else {
		i->i_op = &ipcufs_inode_operations;
		i->i_fop = &ipcufs_file_operations;
		i->i_data.a_ops = &ipcufs_aops;
	}
}

static int ipcufs_create(struct inode *dir, struct dentry *dentry,
			 umode_t mode, bool excl)
{
	struct inode *i;
	int ino;
	struct ipcufs_info_table *it =
	    kzalloc(sizeof(struct ipcufs_info_table), GFP_NOWAIT);
	char *buf = kzalloc(IPCUFS_MAX_PATH_LEN, GFP_NOWAIT);
	char *path_in = dentry_path_raw(dentry, buf, IPCUFS_MAX_PATH_LEN);
	int ret = 0;

	mutex_lock(&flock);

	if (dentry->d_name.len >= sizeof(it->c.icreate.path) - 1) {
		ret = -ENOMEM;
		goto end;
	}
#ifdef IPCUFS_USE_FAT
	if (ipcufs_get_real_filename(dentry->d_sb->s_fs_info,
				     dir->i_ino, dentry->d_name.name)) {
		ret = -EEXIST;
		goto end;
	}
#endif

	pr_debug("%s: '%s', size %lld, parent %ld, type %d\n", __func__,
		 dentry->d_name.name,
		 dir->i_size, dir->i_ino, it->c.icreate.type);

	if ((mode & S_IFMT) == S_IFDIR)
		it->c.icreate.type = 1;

	it->u.action = IFSA_ICREATE;
	it->c.icreate.drive = (u32) dentry->d_sb->s_fs_info;
	strncpy(it->c.icreate.path, path_in, IPCUFS_MAX_PATH_LEN);

	if (ipcufs_transfer((void *)it, sizeof(struct ipcufs_info_table))) {
		ret = -EEXIST;
		goto end;
	}

	if (it->c.icreate.err) {
		ret = -EIO;
		goto end;
	}

	/* okay make the logical inode entry */

	ino = ipcufs_get_fake_ino((u32) dentry->d_sb->s_fs_info,
				  dir->i_ino, dentry->d_name.name);

	i = iget_locked(dir->i_sb, ino);

	if (!i) {
		ret = -ENOMEM;
		goto end;
	}

	i->i_mode = mode;
	__ipcufs_init_ops(i, dentry);

	d_instantiate(dentry, i);
	unlock_new_inode(i);

 end:
	mutex_unlock(&flock);
	kfree(it);
	kfree(buf);

	return ret;
}

/* Do ipcufs protocol side process: call under flock locked */
static int __ipcufs_unlink(struct inode *inode, struct dentry *dentry)
{
	struct ipcufs_info_table *it =
	    kzalloc(sizeof(struct ipcufs_info_table), GFP_NOWAIT);
	char *buf = kzalloc(IPCUFS_MAX_PATH_LEN, GFP_NOWAIT);
	char *path_in = dentry_path_raw(dentry, buf, IPCUFS_MAX_PATH_LEN);
	int ret = 0;


#ifdef IPCUFS_USE_FAT
	char *realname;

	realname = ipcufs_get_real_filename((u32) dentry->d_sb->s_fs_info,
					    inode->i_ino, dentry->d_name.name);

	if (strcmp(realname, dentry->d_name.name)) {
		ret = -EEXIST;
		goto end;
	}

	strtoupper(path_in);

#endif

	it->c.idel.type = d_is_dir(dentry) ? 1 : 0;

	it->u.action = IFSA_IDELETE;
	it->c.idel.drive = (u32) dentry->d_sb->s_fs_info;
	strncpy(it->c.idel.path, path_in, IPCUFS_MAX_PATH_LEN);

	if (ipcufs_transfer((void *)it, sizeof(struct ipcufs_info_table))) {
		ret = -EIO;
		goto end;
	}

	if (it->c.idel.err)
		ret = -EIO;

	if (ipcufs_delete_fake_ino((u32) dentry->d_sb->s_fs_info,
				   inode->i_ino, dentry->d_inode->i_ino)) {
		ret = -EIO;
		goto end;
	}

 end:
	kfree(it);
	kfree(buf);

	return ret;
}

static int ipcufs_unlink(struct inode *inode, struct dentry *dentry)
{
	int ret;

	mutex_lock(&flock);

	ret = __ipcufs_unlink(inode, dentry);
	if (!ret)
		simple_unlink(inode, dentry);

	mutex_unlock(&flock);

	return ret;
}

static int ipcufs_mkdir(struct inode *inode, struct dentry *dentry,
			umode_t mode)
{
	return ipcufs_create(inode, dentry, S_IFDIR | mode, 0);
}

static int ipcufs_rmdir(struct inode *inode, struct dentry *dentry)
{
	int ret;

	if (!simple_empty(dentry))
		return -ENOTEMPTY;

	mutex_lock(&flock);

	ret = __ipcufs_unlink(inode, dentry);
	if (!ret)
		simple_rmdir(inode, dentry);

	mutex_unlock(&flock);

	return ret;
}

static int ipcufs_rename(struct inode *inode, struct dentry *dentry,
			 struct inode *inode_new, struct dentry *dentry_new)
{
	struct ipcufs_info_table *it =
	    kzalloc(sizeof(struct ipcufs_info_table), GFP_NOWAIT);
	char *path_buf = kzalloc(IPCUFS_MAX_PATH_LEN, GFP_NOWAIT);
	char *path_in = dentry_path_raw(dentry, path_buf, IPCUFS_MAX_PATH_LEN);
	char *path_buf_new = kzalloc(IPCUFS_MAX_PATH_LEN, GFP_NOWAIT);
	char *path_in_new =
	    dentry_path_raw(dentry_new, path_buf_new, IPCUFS_MAX_PATH_LEN);
	int ret = 0;
#ifdef IPCUFS_USE_FAT
	char *realname;

	realname = ipcufs_get_real_filename((u32) dentry->d_sb->s_fs_info,
					    inode->i_ino, dentry->d_name.name);
	if (strcmp(realname, dentry->d_name.name)) {
		ret = -EEXIST;
		goto end;
	}

	strtoupper(path_in);

#endif

	mutex_lock(&flock);

	it->u.action = IFSA_IRENAME;
	it->c.iren.drive = (u32) dentry->d_sb->s_fs_info;
	strncpy(it->c.iren.path, path_in, IPCUFS_MAX_PATH_LEN);
	it->c.iren.renamed_drive = (u32) dentry_new->d_sb->s_fs_info;
	strncpy(it->c.iren.renamed_path, path_in_new, IPCUFS_MAX_PATH_LEN);

	if (ipcufs_transfer((void *)it, sizeof(struct ipcufs_info_table))) {
		ret = -EIO;
		goto end;
	}

	if (it->c.iren.err) {
		ret = -EIO;
		goto end;
	}

	if (dentry_new->d_inode) {
		ipcufs_unlink(inode_new, dentry_new);
		if (S_ISDIR(dentry->d_inode->i_mode)) {
			drop_nlink(dentry_new->d_inode);
			drop_nlink(inode);
		}
	} else if (S_ISDIR(dentry->d_inode->i_mode)) {
		drop_nlink(inode);
		inc_nlink(inode_new);
	}

	if (ipcufs_rename_fake_ino((u32) dentry->d_sb->s_fs_info,
				   (u32) dentry_new->d_sb->s_fs_info,
				   inode->i_ino,
				   inode_new->i_ino,
				   dentry->d_inode->i_ino,
				   (const char *)dentry_new->d_name.name)) {
		ret = -EIO;
		goto end;
	}

 end:
	mutex_unlock(&flock);
	kfree(it);
	kfree(path_buf);
	kfree(path_buf_new);

	return ret;
}

static const struct inode_operations ipcufs_dir_inode_operations = {
	.lookup = ipcufs_lookup,
	.create = ipcufs_create,
	.unlink = ipcufs_unlink,
	.mkdir = ipcufs_mkdir,
	.rmdir = ipcufs_rmdir,
	.rename = ipcufs_rename,
};

static int __ipcufs_iget(struct inode *i, u32 drv, const char *path,
			 struct dentry *dentry);

static int ipcufs_update_inode(struct inode *inode, struct dentry *dentry)
{
	u32 drive = (u32) inode->i_sb->s_fs_info;
	char *name = __getname(), *p;
	loff_t size;
	__kernel_time_t mtime;
	int ret;

	if (!name)
		return -ENOMEM;

	mutex_lock(&flock);

	/* Update inode information */
	p = dentry_path_raw(dentry, name, PATH_MAX);
	if (IS_ERR(p)) {
		ret = PTR_ERR(p);
		goto free_out;
	}

	mtime = inode->i_mtime.tv_sec;
	size = inode->i_size;
	ret = __ipcufs_iget(inode, drive, p, dentry);
	if (ret < 0)
		goto free_out;

	/* Invalidate page caches if there is any update */
	if (mtime != inode->i_mtime.tv_sec || size != inode->i_size)
		invalidate_mapping_pages(inode->i_mapping, 0, -1);

free_out:
	mutex_unlock(&flock);

	__putname(name);

	return ret;
}

static int ipcufs_getattr(struct vfsmount *mnt, struct dentry *dentry,
			  struct kstat *stat)
{
	int ret;

	ret = ipcufs_update_inode(d_inode(dentry), dentry);

	/* And fill kstat information by simple_getattr */
	return ret ? ret : simple_getattr(mnt, dentry, stat);
}

static const struct inode_operations ipcufs_inode_operations = {
	.getattr	= ipcufs_getattr,
};

static int __ipcufs_iget_transfer(struct inode *i, u32 drv, const char *path,
				  struct dentry *dentry)
{
	struct ipcufs_info_table *it =
	    kzalloc(sizeof(struct ipcufs_info_table), GFP_NOWAIT);

	it->u.action = IFSA_IGET;
	it->c.iget.drive = drv;
	strncpy(it->c.iget.path, path, IPCUFS_MAX_PATH_LEN - 1);
	it->c.iget.path[sizeof(it->c.iget.path) - 1] = '\0';

	if (ipcufs_transfer((void *)it, sizeof(struct ipcufs_info_table)))
		goto err;

	if (it->c.iget.err)
		goto err;

	i->i_size = it->c.iget.file_length;

	i->i_atime.tv_sec =
	    mktime(IPCUFS_TIME_YEAR_OFFSET + (it->c.iget.a_date >> 9),
		   (it->c.iget.a_date >> 5) & 0xf,
		   it->c.iget.a_date & 0x1f,
		   (it->c.iget.a_time >> 11) & 0x1f,
		   (it->c.iget.a_time >> 5) & 0x3f,
		   (it->c.iget.a_time & 0x1f) * 2);

	i->i_mtime.tv_sec =
	    mktime(IPCUFS_TIME_YEAR_OFFSET + (it->c.iget.date >> 9),
		   (it->c.iget.date >> 5) & 0xf,
		   it->c.iget.date & 0x1f,
		   (it->c.iget.time >> 11) & 0x1f,
		   (it->c.iget.time >> 5) & 0x3f, (it->c.iget.time & 0x1f) * 2);

	i->i_ctime.tv_sec =
	    mktime(IPCUFS_TIME_YEAR_OFFSET + (it->c.iget.date >> 9),
		   (it->c.iget.date >> 5) & 0xf,
		   it->c.iget.date & 0x1f,
		   (it->c.iget.time >> 11) & 0x1f,
		   (it->c.iget.time >> 5) & 0x3f, (it->c.iget.time & 0x1f) * 2);

	if (i->i_state & I_NEW) {
		if (it->c.iget.type)
			i->i_mode = S_IFDIR | 0755;
		else
			i->i_mode = S_IFREG | 0755;
	}

	kfree(it);
	trace_ipcufs_iget(i->i_ino);
	return 0;

err:
	kfree(it);
	return -EIO;
}

static int __ipcufs_iget(struct inode *i, u32 drv, const char *path,
			 struct dentry *dentry)
{
	int ret;

	ret = __ipcufs_iget_transfer(i, drv, path, dentry);
	if (ret)
		return ret;

	if (i->i_state & I_NEW)
		__ipcufs_init_ops(i, dentry);

	return 0;
}

static struct inode *ipcufs_iget(struct super_block *sb, int pos,
				 const char *path, struct dentry *dentry)
{
	struct inode *i;

	i = iget_locked(sb, pos);
	if (!i)
		goto err;

	if (!(i->i_state & I_NEW))
		goto err;

	if (__ipcufs_iget(i, (u32) sb->s_fs_info, path, dentry) < 0)
		goto unlock_err;

	unlock_new_inode(i);

	return i;

 unlock_err:
	unlock_new_inode(i);
 err:
	iput(i);
	return NULL;
}

/* Copy the inode and register it in the dentry */
static struct inode *ipcufs_iget_register(struct inode *inode,
					  struct super_block *sb, int pos,
					  const char *path,
					  struct dentry *dentry)
{
	struct inode *i;

	i = iget_locked(sb, pos);
	if (!i)
		goto err;

	if (!(i->i_state & I_NEW))
		goto err;

	i->i_size = inode->i_size;

	i->i_atime.tv_sec = inode->i_atime.tv_sec;
	i->i_mtime.tv_sec = inode->i_mtime.tv_sec;
	i->i_ctime.tv_sec = inode->i_ctime.tv_sec;

	if (i->i_state & I_NEW) {
		i->i_mode = inode->i_mode;
		__ipcufs_init_ops(i, dentry);
	}

	unlock_new_inode(i);

	return i;

 err:
	iput(i);
	return NULL;
}

static void ipcufs_umount_begin(struct super_block *sb)
{
	struct ipcufs_info_table *it =
	    kzalloc(sizeof(struct ipcufs_info_table), GFP_NOWAIT);

	mutex_lock(&flock);

	if (fake_file[(u32) sb->s_fs_info]) {

		it->u.action = IFSA_UMOUNTING;
		it->c.imount.drive = (u32) sb->s_fs_info;

		if (ipcufs_transfer((void *)it, sizeof(*it)))
			pr_err("%s: problem sending umount\n", __func__);

		ipcufs_free_drive((u32) sb->s_fs_info);

		sb->s_fs_info = NULL;
	}

	bdi_destroy(sb->s_bdi);

	if (ipcufs_ipcu_uninit())
		pr_err("%s: uninit ipcu\n", __func__);

	mutex_unlock(&flock);

	kfree(it);

}

static int ipcufs_statfs(struct dentry *dentry, struct kstatfs *buf)
{
	struct ipcufs_info_table *it =
	    kzalloc(sizeof(struct ipcufs_info_table), GFP_NOWAIT);

	mutex_lock(&flock);

	it->u.action = IFSA_ISTATFS;
	it->c.istat.drive = (u32) dentry->d_sb->s_fs_info;

	if (ipcufs_transfer((void *)it, sizeof(*it)))
		pr_err("%s: problem getting file system status\n", __func__);

	buf->f_type = dentry->d_sb->s_magic;
	buf->f_bsize = it->c.istat.blksize;
	buf->f_blocks = it->c.istat.blocks;
	buf->f_bfree = it->c.istat.bfree;
	buf->f_bavail = it->c.istat.bfree;
	buf->f_files = IPCUFS_MAX_INODE_NUM - 1;
	buf->f_ffree = ipcufs_count_free_ino((u32)dentry->d_sb->s_fs_info);
#ifdef IPCUFS_USE_FAT
	buf->f_namelen = 12;
#else
	buf->f_namelen = IPCUFS_MAX_NAME_LEN;
#endif

	mutex_unlock(&flock);
	kfree(it);

	return 0;
}

static struct super_operations ipcufs_ops = {
	.drop_inode = generic_delete_inode,
	.umount_begin = ipcufs_umount_begin,
	.statfs = ipcufs_statfs,
};

/* Parse the mount option */
static int ipcufs_parse_option(const char *p)
{
	char *end;
	u32 s = 0;
	u32 temp;

	if (p) {
		while (*p && *p != ',' && *p != '=')
			p++;
		if (*p != '=') {
			pr_info("%s: mount option fs=<num>,<inode_val> needed\n", __func__);
			return -EINVAL;
		}
		p++;

		/* get media offset */
		s = simple_strtoul(p, &end, 0);
		if (s < 0 || s >= IPCUFS_MAX_DRV_NUM) {
			pr_info("%s: mount option <num> invalid\n", __func__);
			return -EINVAL;
		}

		/* get inode nums*/
		while (*p && *p != ',')
			p++;

		inode_num = IPCUFS_MAX_INODE_NUM;

		if (*p == ',') {
			p++;
			temp = simple_strtoul(p, &end, 0);
			if (temp <= 0 || temp > IPCUFS_MAX_INODE_NUM) {
				pr_info("%s: mount option <inode_val> invalid\n", __func__);
				return -EINVAL;
			}
			inode_num = temp;
		}
	}

	return (int)s;
}

static int ipcufs_fill_sb(struct super_block *sb, void *data, int silent)
{
	struct inode *root;
	int s = 0;

	s = ipcufs_parse_option((const char *)data);
	if (s < 0)
		return s;

	sb->s_fs_info = (void *)s;
	sb->s_op = &ipcufs_ops;
	sb->s_maxbytes = 0x7fffffffffffffffLL;

	sb->s_blocksize = 4096;
	sb->s_blocksize_bits = 12;

	root = ipcufs_iget(sb, 1, "/", NULL);

	if (!root)
		return -EINVAL;

	sb->s_root = d_make_root(root);

	if (!sb->s_root)
		return -EINVAL;

	/* set bdi information */
	sb->s_bdi = &ipcufs_bdi[s];
	sb->s_bdi->ra_pages = (VM_MAX_READAHEAD * 1024)/PAGE_CACHE_SIZE;
	sb->s_bdi->capabilities = BDI_CAP_NO_ACCT_AND_WRITEBACK;

	return 0;
}

static struct dentry *ipcufs_mount(struct file_system_type *fs_type,
				   int flags, const char *dev_name, void *data)
{
	struct ipcufs_info_table *it =
	    kzalloc(sizeof(struct ipcufs_info_table), GFP_NOWAIT);
	struct dentry *ret = ERR_PTR(-EINVAL);
	int s = 0;
	int rc = -ENOMEM;

	mutex_lock(&flock);

	if (ipcufs_ipcu_init())
		goto err1;

	s = ipcufs_parse_option((const char *)data);
	if (s < 0) {
		ret = ERR_PTR(s);
		goto err1;
	}

	rc = bdi_setup_and_register(&ipcufs_bdi[s], "ipcufs");
	if (rc)
		goto err1;

	if (!fake_file[s]) {
		ipcufs_alloc_drive(s);

		it->u.action = IFSA_MOUNTING;
		it->c.imount.drive = s;

		if (ipcufs_transfer
		    ((void *)it, sizeof(struct ipcufs_info_table)))
			goto err3;

		if (it->c.imount.err)
			goto err3;

		ret = mount_ns(fs_type, flags, data, ipcufs_fill_sb);

		if (IS_ERR(ret))
			goto err3;
	} else {
		ret = ERR_PTR(-EBUSY);
		goto err2;
	}

	mutex_unlock(&flock);
	kfree(it);
	return ret;

 err3:
	ipcufs_free_drive(s);
 err2:
	bdi_destroy(&ipcufs_bdi[s]);
 err1:
	ipcufs_ipcu_uninit();
	mutex_unlock(&flock);
	kfree(it);
	return ret;
}

static struct file_system_type ipcufs_type = {
	.name = "ipcufs",
	.mount = ipcufs_mount,
	.kill_sb = kill_anon_super,
	.fs_flags = FS_REQUIRES_DEV,
};

static int sn_ipcufs_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	void __iomem *common_mem;
	u32 offset;
	u32 shm_start_addr;

	if (of_property_read_u32(dev->of_node, "ipcu_unit", &ipcu_unit)) {
		dev_err(dev, "%s:%d IPCU Unit Number is not provided\n",
			__func__, __LINE__);
		return -EINVAL;
	}

	if (of_property_read_u32_index(dev->of_node, "ipcu_ch", 0, &send_ch)) {
		dev_err(dev, "%s:%d IPCU Send Channel is not provided\n",
			__func__, __LINE__);
		return -EINVAL;
	}

	if (of_property_read_u32_index(dev->of_node, "ipcu_ch", 1, &recv_ch)) {
		dev_err(dev, "%s:%d IPCU Receive Channel is not provided\n",
			__func__, __LINE__);
		return -EINVAL;
	}

	common_mem = shared_mem_get_mem(E_SHARED_MEM_BUFFER);
	if (!common_mem) {
		dev_err(dev, "get shared_mem failed.\n");
		return -EINVAL;
	}

	send_dev.dest_unit = ipcu_unit;
	send_dev.dest_channel = send_ch;

	recv_dev.dest_unit = ipcu_unit;
	recv_dev.dest_channel = recv_ch;

	offset = IPCU_COMMEM_DOMAIN_SIZE * send_ch;
	shm_start_addr = ioread32(common_mem) + offset;

	shm_paddr = shm_start_addr;
	shm = ioremap(shm_paddr, IPCUFS_TABLE_BUF_SIZE);

	page_shm_paddr = shm_start_addr + IPCUFS_TABLE_BUF_SIZE;
	page_shm = ioremap(page_shm_paddr, IPCUFS_PAGE_BUF_SIZE);

	dio_buf = kmalloc(IPCUFS_DIO_BUF_SIZE, GFP_KERNEL);

	mutex_init(&mlock);
	mutex_init(&flock);

	if (!shm)
		return -ENOMEM;

	ret = register_filesystem(&ipcufs_type);

	fake_file =
	    kzalloc(sizeof(struct fake_inode *) * IPCUFS_MAX_DRV_NUM,
		    GFP_NOWAIT);

	ipcufs_wq = create_singlethread_workqueue("ipcufs_recv");
	INIT_WORK(&work, ipcufs_work_handler);

	/* initialize backing dev info */
	memset(&ipcufs_bdi, 0x0, sizeof(ipcufs_bdi));

	return ret;
}

static int sn_ipcufs_remove(struct platform_device *pdev)
{

	unregister_filesystem(&ipcufs_type);

	iounmap(shm);
	iounmap(page_shm);
	kfree(dio_buf);
	kfree(fake_file);

	return 0;
}

static const struct of_device_id sn_ipcufs_rtos_id[] = {
	{.compatible = "socionext,ipcufs-rtos"},
	{},
};

MODULE_DEVICE_TABLE(of, sn_ipcufs_rtos_id);

static struct platform_driver sn_ipcufs_rtos_driver = {
	.probe = sn_ipcufs_probe,
	.remove = sn_ipcufs_remove,
	.driver = {
		   .name = "ipcufs-rtos",
		   .of_match_table = sn_ipcufs_rtos_id,
		   },
};

module_platform_driver(sn_ipcufs_rtos_driver);

MODULE_LICENSE("GPL");
