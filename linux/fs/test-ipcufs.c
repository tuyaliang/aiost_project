/*
 * ipcufs remote filesystem proxy for Socionext
 *
 * Copyright (C) 2015 Linaro, Ltd
 * Andy Green <andy.green@linaro.org>
 *
 * Limitations:
 *
 *   - only understands files and directories (no links)
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/pagemap.h>
#include <linux/uaccess.h>
#include <linux/of_platform.h>
#include <linux/mailbox_client.h>
#include <linux/dma-mapping.h>
#include <linux/buffer_head.h>
#include <linux/slab.h>

#include <soc/mb86s7x/ipcu.h>

#include "test-ipcufs.h"

#define IPCUFS_END_POS 0x7fffffff

static struct inode *ipcufs_iget(struct super_block *sb, int pos);
static const struct inode_operations ipcufs_dir_inode_operations;

#define IPCUFS_LOOPBACK

#ifdef IPCUFS_LOOPBACK
/***** START: move this code to RTOS ******/

/*
 * "Remote" canned filesystem
 * This represents the RTOS side filesystem
 *
 * Replace this section with code to report and manipulate the actual
 * remote filesystem after testing with the canned one
 *
 * Linux side has NO KNOWLEDGE of this data structure directly, so it
 * can be anything needed.
 *
 * Linux side does not try to understand the inode numbers.  He just
 * gives back the inode numbers he was given from the remote side in
 * other commands to the remote side.  But there are two simple rules:
 *
 * 1) inode 0 is reserved and can't be used directly (it's used in
 *    lists of inodes to mean "no inode")
 *
 * 2) inode 1 is the "root directory" of the filesystem
 *
 * all other inodes can be whatever will fit in a u64, ie, private
 * inode numbers or pointers from RTOS filesystem code.
 */

struct remote_inode {
	char *name;
	u64 i_target;		/* first inode inside directory or 0 */
	u64 i_sibling;		/* next inode in same directory or 0 */
	u32 type;		/* b0 = 0 = normal file, 1 = directory */
	u64 length;		/* payload length or 0 */
	void *payload;		/* pointer to payload (file contents) */
	u64 mtime_s;		/* modified time: secs since 1970-1-1 0:0 */
};

struct remote_inode init_remote_fs[] = {
	[1] = {
	       .name = "/",
	       .i_target = 2,
	       .type = IPCUFS_REMOTE_TYPE_DIR,
	       .mtime_s = 1435801234,
	       },
	[2] = {
	       .name = "test1",
	       .i_sibling = 3,
	       .length = 6,
	       .payload = "hello\n",
	       .mtime_s = 1435805678,
	       },
	[3] = {
	       .name = "testdir",
	       .i_target = 5,
	       .i_sibling = 4,
	       .type = IPCUFS_REMOTE_TYPE_DIR,
	       .mtime_s = 1435802048,
	       },
	[4] = {
	       .name = "test2",
	       .length = 12,
	       .payload = "hello again\n",
	       .mtime_s = 1435305678,
	       },
	[5] = {
	       .name = "test3",
	       .i_sibling = 6,
	       .length = 16,
	       .payload = "I'm in a subdir\n",
	       .mtime_s = 1435805678,
	       },
	[6] = {
	       .name = "hugefile",
	       .length = SZ_128M,
	       .payload = NULL,
	       .mtime_s = 1435805678,
	       },
};

#define NUM_FILESYSTEMS 3

static const char *filesystem_names[] = {
	"sd",
	"nf1",
	"nf2",
};

struct remote_inode remote_fs[NUM_FILESYSTEMS][20];

static void ipcufs_init_on_mount(int filesystem_index)
{
	int n;

	memset(&remote_fs[filesystem_index][0], 0, sizeof remote_fs[0]);

	for (n = 1; n < ARRAY_SIZE(init_remote_fs); n++) {
		remote_fs[filesystem_index][n] = init_remote_fs[n];
		if (remote_fs[filesystem_index][n].name)
			remote_fs[filesystem_index][n].name =
			    kstrdup(init_remote_fs[n].name, GFP_KERNEL);
		if (remote_fs[filesystem_index][n].payload)
			remote_fs[filesystem_index][n].payload =
			    kstrdup(init_remote_fs[n].payload, GFP_KERNEL);
	}
}

static int which_filesystem(const char *name)
{
	int n;

	for (n = 0; n < ARRAY_SIZE(filesystem_names); n++)
		if (strcmp(name, filesystem_names[n]) == 0)
			return n;

	return -1;
}

static bool is_valid_inode(u64 n)
{
	return n && n < ARRAY_SIZE(remote_fs[0]);
}

static int find_unused_inode(struct remote_inode *ri)
{
	int n = 1;

	while (n < ARRAY_SIZE(remote_fs[0])) {

		if (!ri[n].name)
			return n;
		n++;
	}

	/* there are no free inodes */

	return 0;
}

/* this represents the remote filesystem handler, replace
 * with code in RTOS that receives remote IPCU, processes it and responds
 * over IPCU
 */

static void remote_fs_handler(struct ipcufs_transport *it)
{
	int filesystem_index;
	struct remote_inode *ri;
	int n;
	u64 p, *pp;
	u8 *v;

	it->result = 0;

	it->fs_name[ARRAY_SIZE(it->fs_name) - 1] = '\0';
	filesystem_index = which_filesystem(&it->fs_name[0]);
	if (filesystem_index < 0) {
		pr_err("unknown filesystem name\n");
		goto fail;
	}
	ri = remote_fs[filesystem_index];
	pr_debug("filesystem context %s / index %d\n", it->fs_name,
		 filesystem_index);

	switch (it->action) {
	case IFSA_IGET:
		if (!is_valid_inode(it->u.iget.inode))
			goto bad_inode;
		it->u.iget.inode_length = ri[it->u.iget.inode].length;
		it->u.iget.type = ri[it->u.iget.inode].type;
		it->u.iget.mtime_ns = it->u.iget.atime_ns =
		    it->u.iget.ctime_ns = 0;
		/* in this pretend fs, only modified time kept */
		it->u.iget.mtime_s = it->u.iget.atime_s =
		    it->u.iget.ctime_s = ri[it->u.iget.inode].mtime_s;
		break;
	case IFSA_READPAGE:
		if (!is_valid_inode(it->u.page.inode))
			goto bad_inode;

		for (n = 0; n < it->u.page.count_pages; n++) {
			v = phys_to_virt(it->u.page.pa);

			if (ri[it->u.page.inode].payload)
				memcpy(v, ri[it->u.page.inode].payload,
				       it->u.page.len);
			/* else it's hugefile, currently no payload */
		}

		/* remote cpu needs to flush this written cache region */

		break;

	case IFSA_WRITEPAGE:
		pr_debug("%s: IFSA_WRITEPAGE: inode %lld, pos %lld, len %d\n",
			 __func__, it->u.page.inode, it->u.page.pos,
			 it->u.page.len);
		v = phys_to_virt(it->u.page.pa);
		pr_info("%02X %02X\n", v[0], v[1]);
		break;

	case IFSA_READDIR:
		if (!is_valid_inode(it->u.rdir.inode))
			goto bad_inode;

		pr_debug("%s: IFSA_READDIR: ctxpos %lld\n", __func__,
			 it->u.rdir.ctxpos);

		p = it->u.rdir.inode;
		if (it->u.rdir.ctxpos == 2)
			/* if at the directory inode, convert to first child */
			p = ri[p].i_target;

		if (!p) {
			it->result = 1;
			break;
		}
		n = strlen(ri[p].name);
		if (n > sizeof(it->u.rdir.name - 1))
			n = sizeof(it->u.rdir.name) - 1;
		if (n) {
			strncpy(it->u.rdir.name, ri[p].name, n);
			it->u.rdir.name[n] = '\0';
		}
		it->u.rdir.name_len = n;
		it->u.rdir.type = ri[p].type;
		it->u.rdir.i_sibling = ri[p].i_sibling;
		break;

	case IFSA_LOOKUP:
		if (!is_valid_inode(it->u.lookup.inode_in))
			goto bad_inode;

		p = it->u.lookup.inode_in;
		p = ri[p].i_target;
		if (!p) {
			it->result = 1;
			break;
		}
		do {
			if (!strcmp(ri[p].name, it->u.lookup.name)) {
				it->u.lookup.inode_out = p;
				goto fin;
			}

			p = ri[p].i_sibling;
		} while (p);
		it->result = 1;
		break;

	case IFSA_ICREATE:
		if (!is_valid_inode(it->u.icreate.i_parent))
			goto bad_inode;
		p = find_unused_inode(ri);
		if (!p)
			goto bad_inode;

		pr_info
		    ("%s: IFSA_ICREATE: found empty inode %lld, parent=%lld, type=%d\n",
		     __func__, p, it->u.icreate.i_parent,
		     it->u.icreate.idata.type);

		/* give linux the new inode number */
		it->u.icreate.idata.inode = p;

		/* set up the inode with what was sent */
		ri[p].name = kstrdup(it->u.icreate.name, GFP_KERNEL);
		ri[p].mtime_s = it->u.icreate.idata.mtime_s;
		ri[p].type = it->u.icreate.idata.type;

		/* stitch this inode into the parent's list */
		pr_debug("parent target (new sibling) = %lld\n",
			 ri[it->u.icreate.i_parent].i_target);
		ri[p].i_sibling = ri[it->u.icreate.i_parent].i_target;
		ri[it->u.icreate.i_parent].i_target = p;
		ri[p].i_target = 0;
		ri[p].payload = NULL;
		ri[p].length = 0;
		break;

	case IFSA_IDELETE:
		if (!is_valid_inode(it->u.idel.inode))
			goto bad_inode;
		if (!is_valid_inode(it->u.idel.inode_parent))
			goto bad_inode;

		/* walk the parent dir siblings until we find the link */
		pp = &ri[it->u.idel.inode_parent].i_target;

		while (*pp && *pp != it->u.idel.inode)
			pp = &ri[*pp].i_sibling;

		if (!*pp) {
			it->result = 1;
			break;
		}

		/* snip him out of linked-list */
		*pp = ri[it->u.idel.inode].i_sibling;

		/* remove his payload */
		kfree(ri[it->u.idel.inode].payload);

		/* free up his name and mark not in use */
		kfree(ri[it->u.idel.inode].name);
		ri[it->u.idel.inode].name = NULL;

		break;

	case IFSA_IRENAME:
		pr_info("%s: IFSA_RENAME: %s to %s\n", __func__,
			ri[it->u.iren.inode].name, it->u.iren.name);
		kfree(ri[it->u.iren.inode].name);
		ri[it->u.iren.inode].name =
		    kstrdup(it->u.iren.name, GFP_KERNEL);
		break;
	case IFSA_MOUNTING:
		pr_info("remote side seeing mounting %s = index %d\n",
			it->fs_name, filesystem_index);
		ipcufs_init_on_mount(filesystem_index);
		break;
	case IFSA_UMOUNTING:
		pr_info("remote side seeing umounting\n");
		break;
	}

	goto fin;

 bad_inode:
	pr_err("%s: action %d: bad inode\n", __func__, it->action);
 fail:
	it->result = -EINVAL;

 fin:
	/* remote cpu needs to flush *it cache region */

	return;			/* keep gcc happy - remove when cache flush code added */
}

/***** END: move this code to RTOS *****/

/*
 * All the linux-side transport goes through here
 */

static int ipcufs_transfer_request(const void *fs_name,
				   struct ipcufs_transport *it)
{
	strncpy(it->fs_name, (const char *)fs_name, ARRAY_SIZE(it->fs_name));

	/*
	 * right now we just call the "remote" side directly
	 * this will be replaced by IPCU kernel API
	 * where this struct is passed as part of some other protocol
	 *
	 * serialization enforcement mutex needs dealing with via IPCU code
	 */
	remote_fs_handler(it);

	return it->result;
}

#else

static struct mbox_client cl;
static struct mbox_chan *chan;
static struct mutex mlock;
static unsigned dest_cpu;	/* destination cpu i/f */
static void *shm;
static dma_addr_t shm_paddr;

/* From IPCU_abstract_20150630.pdf */
struct ipcu_payload {
	u32 id;			/* command ID */
	u32 addr;		/* physical address of payload */
	u32 len;		/* length (in bytes) of payload */
	u32 flag;		/* continuous or sync transfer */
	u32 unused[5];
} __packed;

/* Block of outer command codes with ipcufs ones starting there */
#define IPCUCMD_IPCUFS(cmd) (0x1234 + cmd)

static void ipcufs_callback(struct mbox_client *c, void *m)
{
	struct ipcu_mssg *r = m;
	int i;

	if (r->mask != dest_cpu) {
		pr_err("Message from unknown cpu i/f (%x)\n", r->mask);
		return;
	}

	pr_debug("MTP Event From RTOS -> \n");
	for (i = 0; i < 9; i++)
		pr_debug("%x ", r->data[i]);
}

static int ipcufs_transfer(u32 cmd, void *p, int len)
{
	struct ipcu_mssg mssg;
	struct ipcu_payload *pl;
	int ret;

	memset(&mssg, 0, sizeof(mssg));
	pl = (struct ipcu_payload *)&mssg;
	pl->id = cmd;
	pl->addr = shm_paddr;
	pl->len = len;
	mssg.mask = dest_cpu;

	mutex_lock(&mlock);

	if (p)
		memcpy(shm, p, len);
	wmb();

	ret = mbox_send_message(chan, &mssg);
	if (ret >= 0) {
		ret = 0;
		if (p)
			memcpy(p, shm, len);
	}

	mutex_unlock(&mlock);

	return ret;
}

#endif

/*
 * and the rest of this is generic for Linux and shouldn't need changing --->
 */

static struct dentry *ipcufs_lookup(struct inode *dir, struct dentry *dentry,
				    unsigned int flags)
{
	struct inode *inode = NULL;
	struct ipcufs_transport it;

	if (dentry->d_name.len >= sizeof(it.u.lookup.name))
		goto fail;

	pr_debug("%s: dir->i_ino=%lld, '%s'\n", __func__, (u64) dir->i_ino,
		 it.u.lookup.name);

	it.action = IFSA_LOOKUP;
	it.u.lookup.inode_in = dir->i_ino;
	strncpy(it.u.lookup.name, dentry->d_name.name,
		sizeof(it.u.lookup.name) - 1);
	it.u.lookup.name[sizeof(it.u.lookup.name) - 1] = '\0';
	it.u.lookup.name_len = dentry->d_name.len;
#ifdef IPCUFS_LOOPBACK
	if (ipcufs_transfer_request(dir->i_sb->s_fs_info, &it))
#else
	if (ipcufs_transfer
	    (IFSA_LOOKUP, (void *)&it.u.lookup, sizeof(it.u.lookup)))
#endif
		goto fail;

	inode = ipcufs_iget(dir->i_sb, it.u.lookup.inode_out);

 fail:
	d_add(dentry, inode);

	return 0;
}

static int ipcufs_readdir(struct file *file, struct dir_context *ctx)
{
	struct inode *i = file_inode(file);
	struct ipcufs_transport it;
	int p = i->i_ino;
	int dtype;

	pr_debug("%s: ctx->pos=%lld, p=%d\n", __func__, (u64) ctx->pos, p);

	if (ctx->pos == IPCUFS_END_POS)
		return 0;

	if (ctx->pos < 2)
		if (!dir_emit_dots(file, ctx))
			return 0;

	do {
		it.action = IFSA_READDIR;
		it.u.rdir.inode = p;
		it.u.rdir.ctxpos = ctx->pos;
#ifdef IPCUFS_LOOPBACK
		if (ipcufs_transfer_request
		    (file_inode(file)->i_sb->s_fs_info, &it))
#else
		if (ipcufs_transfer
		    (IFSA_READDIR, (void *)&it.u.rdir, sizeof(it.u.rdir)))
#endif
			break;

		dtype = S_IFREG | 0644;
		if (it.u.rdir.type & IPCUFS_REMOTE_TYPE_DIR)
			dtype = S_IFDIR | 0755;

		pr_debug("  %s, sib %lld\n", it.u.rdir.name,
			 it.u.rdir.i_sibling);

		if (!dir_emit(ctx, it.u.rdir.name, it.u.rdir.name_len,
			      p, dtype)) {
			pr_debug("dir_emit fails\n");
			return -EINVAL;
		}

		p = it.u.rdir.i_sibling;
		ctx->pos = p + 1;
	} while (p);

	ctx->pos = IPCUFS_END_POS;

	return 0;
}

static const struct file_operations ipcufs_dir_operations = {
	.read = generic_read_dir,
	.iterate = ipcufs_readdir,
	.llseek = default_llseek,
};

struct filler_args {
	struct ipcufs_transport it;
	struct scatterlist *slist;
};

static void _ipcufs_clean_page(struct page *page, int ret)
{
	if (!ret)
		SetPageUptodate(page);
	else
		SetPageError(page);

	flush_dcache_page(page);
	kunmap(page);
	unlock_page(page);
}

static int _ipcufs_readpages_transfer(const char *fs,
				      struct ipcufs_transport *it,
				      struct scatterlist *slist,
				      int count_pages)
{
	int ret = 0, n;

#ifdef IPCUFS_LOOPBACK
	if (ipcufs_transfer_request(fs, it))
#else
	if (ipcufs_transfer(IFSA_READPAGE,
			    (void *)&it->u.page, sizeof(it->u.page)))
#endif
		ret = -EIO;
	for (n = 0; n < count_pages; n++)
		_ipcufs_clean_page(sg_page(slist + n), ret);

	return ret;
}

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

static int _ipcufs_rp_filler(void *v, struct page *page)
{
	struct filler_args *a = v;
	struct inode *inode = page->mapping->host;
	void *buf = kmap(page);
	loff_t len = _ipcufs_fillsize(inode, page);

	if (!buf) {
		pr_err("NULL buf\n");
		return -ENOMEM;
	}

	sg_set_page(a->slist + a->it.u.page.count_pages, page, len, 0);
	a->it.u.page.p[a->it.u.page.count_pages].pa =
	    sg_phys(a->slist + a->it.u.page.count_pages);
	a->it.u.page.p[a->it.u.page.count_pages++].len = len;

	return 0;
}

static int ipcufs_readpage(struct file *file, struct page *page)
{
	struct ipcufs_transport it;
	struct inode *inode = page->mapping->host;
	struct scatterlist slist;

	it.action = IFSA_READPAGE;
	it.u.page.inode = inode->i_ino;
	it.u.page.count_pages = 1;
	kmap(page);
	it.u.page.p[0].pa = __pfn_to_phys(page_to_pfn(page));
	it.u.page.p[0].len = _ipcufs_fillsize(inode, page);

	sg_set_page(&slist, page, it.u.page.p[0].len, 0);
	return _ipcufs_readpages_transfer(file_inode(file)->i_sb->s_fs_info,
					  &it, &slist, 1);
}

static int ipcufs_readpages(struct file *file, struct address_space *mapping,
			    struct list_head *pages, unsigned nr_pages)
{
	struct filler_args a;
	struct inode *i;
	int ret;

	if (nr_pages > ARRAY_SIZE(a.it.u.page.p))
		return -EINVAL;

	a.slist = kcalloc(nr_pages, sizeof(struct scatterlist), GFP_KERNEL);

	i = file_inode(file);

	a.it.action = IFSA_READPAGE;
	a.it.u.page.inode = i->i_ino;
	a.it.u.page.count_pages = 0;

	sg_init_table(a.slist, nr_pages);

	ret = read_cache_pages(mapping, pages, _ipcufs_rp_filler, &a);
	if (ret) {
		pr_err("read_cache_pages returned %d\n", ret);
		goto bail;
	}
	if (!a.it.u.page.count_pages) {
		ret = 0;
		goto bail;
	}

	dma_map_sg(NULL, a.slist, nr_pages, DMA_FROM_DEVICE);

	_ipcufs_readpages_transfer(file_inode(file)->i_sb->s_fs_info,
				   &a.it, a.slist, nr_pages);

	dma_unmap_sg(NULL, a.slist, nr_pages, DMA_FROM_DEVICE);

 bail:
	kfree(a.slist);

	return ret;
}

static int ipcufs_write_end(struct file *file, struct address_space *mapping,
			    loff_t pos, unsigned len, unsigned copied,
			    struct page *page, void *fsdata)
{
	struct ipcufs_transport it;
	struct inode *i = file_inode(file);

	it.action = IFSA_WRITEPAGE;
	it.u.page.inode = i->i_ino;
	it.u.page.pa = page_to_phys(page);
	it.u.page.pos = pos;
	it.u.page.len = len;

#ifdef IPCUFS_LOOPBACK
	if (ipcufs_transfer_request(i->i_sb->s_fs_info, &it))
#else
	if (ipcufs_transfer(IFSA_WRITEPAGE, (void *)&it.u.page,
			    sizeof(it.u.page)))
#endif
		return -EIO;

	return simple_write_end(file, mapping, pos, len, copied, page, fsdata);
}

static const struct address_space_operations ipcufs_aops = {
	.readpage = ipcufs_readpage,
	.readpages = ipcufs_readpages,
	.write_begin = simple_write_begin,
	.write_end = ipcufs_write_end,
};

static const struct file_operations ipcufs_file_operations = {
	.llseek = generic_file_llseek,
	.read_iter = generic_file_read_iter,
	.mmap = generic_file_readonly_mmap,
	.splice_read = generic_file_splice_read,
	.write_iter = generic_file_write_iter,
};

static int ipcufs_create(struct inode *dir, struct dentry *dentry,
			 umode_t mode, bool excl)
{
	struct inode *i;
	struct ipcufs_transport it;

	it.action = IFSA_ICREATE;
	it.u.icreate.idata.inode = 0;

	it.u.icreate.idata.inode_length = dir->i_size;

	it.u.icreate.idata.type = 0;
	if ((mode & S_IFMT) == S_IFDIR)
		it.u.icreate.idata.type = IPCUFS_REMOTE_TYPE_DIR;

	it.u.icreate.idata.mtime_s = dir->i_mtime.tv_sec;
	it.u.icreate.idata.atime_s = dir->i_atime.tv_sec;
	it.u.icreate.idata.ctime_s = dir->i_ctime.tv_sec;
	it.u.icreate.idata.mtime_ns = dir->i_mtime.tv_nsec;
	it.u.icreate.idata.atime_ns = dir->i_atime.tv_nsec;
	it.u.icreate.idata.ctime_ns = dir->i_ctime.tv_nsec;

	if (dentry->d_name.len >= sizeof(it.u.icreate.name) - 1)
		return -ENOMEM;
	strcpy(it.u.icreate.name, dentry->d_name.name);
	it.u.icreate.name_len = dentry->d_name.len;
	it.u.icreate.i_parent = dir->i_ino;

	pr_debug("%s: '%s', size %lld, parent %ld, type %d\n", __func__,
		 dentry->d_name.name,
		 dir->i_size, dir->i_ino, it.u.icreate.idata.type);
#ifdef IPCUFS_LOOPBACK
	if (ipcufs_transfer_request(dir->i_sb->s_fs_info, &it))
#else
	if (ipcufs_transfer(IFSA_ICREATE, (void *)&it.u.icreate,
			    sizeof(it.u.icreate)))
#endif
		return -EIO;

	/* okay make the logical inode entry */

	i = iget_locked(dir->i_sb, it.u.icreate.idata.inode);
	if (!i)
		return -ENOMEM;

	if (!(i->i_state & I_NEW)) {
		pr_err("%s: new inode wasn't new\n", __func__);
		return -EBUSY;
	}
	if ((mode & S_IFMT) == S_IFDIR) {
		i->i_op = &ipcufs_dir_inode_operations;
		i->i_fop = &ipcufs_dir_operations;
	} else {
		i->i_fop = &ipcufs_file_operations;
		i->i_data.a_ops = &ipcufs_aops;
	}
	i->i_mode = mode;

	d_instantiate(dentry, i);
	unlock_new_inode(i);

	return 0;
}

static int ipcufs_unlink(struct inode *inode, struct dentry *dentry)
{
	struct ipcufs_transport it;

	it.action = IFSA_IDELETE;
	it.u.idel.inode = dentry->d_inode->i_ino;
	it.u.idel.inode_parent = inode->i_ino;
#ifdef IPCUFS_LOOPBACK
	if (ipcufs_transfer_request(inode->i_sb->s_fs_info, &it))
#else
	if (ipcufs_transfer(IFSA_IDELETE, (void *)&it.u.idel,
			    sizeof(it.u.idel)))
#endif
		return -EIO;

	drop_nlink(dentry->d_inode);
	dput(dentry);

	return 0;
}

static int ipcufs_mkdir(struct inode *inode, struct dentry *dentry,
			umode_t mode)
{
	return ipcufs_create(inode, dentry, S_IFDIR | mode, 0);
}

static int ipcufs_rmdir(struct inode *inode, struct dentry *dentry)
{
	int err;

	if (!simple_empty(dentry))
		return -ENOTEMPTY;

	drop_nlink(dentry->d_inode);
	ipcufs_unlink(inode, dentry);
	drop_nlink(inode);

	return err;
}

static int ipcufs_rename(struct inode *inode, struct dentry *dentry,
			 struct inode *inode_new, struct dentry *dentry_new)
{
	struct ipcufs_transport it;

	it.action = IFSA_IRENAME;
	it.u.iren.inode = dentry->d_inode->i_ino;

	if (dentry->d_name.len >= sizeof(it.u.iren.name) - 1)
		return -ENOMEM;
	strcpy(it.u.iren.name, dentry_new->d_name.name);
	it.u.iren.name_len = dentry_new->d_name.len;
#ifdef IPCUFS_LOOPBACK
	if (ipcufs_transfer_request(inode->i_sb->s_fs_info, &it))
#else
	if (ipcufs_transfer(IFSA_IRENAME, (void *)&it.u.iren,
			    sizeof(it.u.iren)))
#endif
		return -EIO;

	return 0;
}

static const struct inode_operations ipcufs_dir_inode_operations = {
	.lookup = ipcufs_lookup,
	.create = ipcufs_create,
	.unlink = ipcufs_unlink,
	.mkdir = ipcufs_mkdir,
	.rmdir = ipcufs_rmdir,
	.rename = ipcufs_rename,
};

static struct inode *ipcufs_iget(struct super_block *sb, int pos)
{
	struct inode *i;
	struct ipcufs_transport it;

	pr_debug("%s: pos %d\n", __func__, pos);

	i = iget_locked(sb, pos);
	if (!i)
		return ERR_PTR(-ENOMEM);

	if (!(i->i_state & I_NEW))
		return i;

	it.action = IFSA_IGET;
	it.u.iget.inode = pos;
#ifdef IPCUFS_LOOPBACK
	if (ipcufs_transfer_request(sb->s_fs_info, &it))
#else
	if (ipcufs_transfer(IFSA_IGET, (void *)&it.u.iget, sizeof(it.u.iget)))
#endif
		return ERR_PTR(-EIO);

	i->i_size = it.u.iget.inode_length;
	if (it.u.iget.type & IPCUFS_REMOTE_TYPE_DIR) {
		i->i_mode = S_IFDIR | 0644;
		i->i_op = &ipcufs_dir_inode_operations;
		i->i_fop = &ipcufs_dir_operations;
	} else {
		i->i_mode = S_IFREG | 0644;
		i->i_fop = &ipcufs_file_operations;
		i->i_data.a_ops = &ipcufs_aops;
	}

	i->i_mtime.tv_sec = it.u.iget.mtime_s;
	i->i_atime.tv_sec = it.u.iget.atime_s;
	i->i_ctime.tv_sec = it.u.iget.ctime_s;
	i->i_mtime.tv_nsec = it.u.iget.mtime_ns;
	i->i_atime.tv_nsec = it.u.iget.atime_ns;
	i->i_ctime.tv_nsec = it.u.iget.ctime_ns;

	set_nlink(i, 1);

	unlock_new_inode(i);

	return i;
}

static void ipcufs_umount_begin(struct super_block *sb)
{
	struct ipcufs_transport it;

	it.action = IFSA_UMOUNTING;
#ifdef IPCUFS_LOOPBACK
	if (ipcufs_transfer_request(sb->s_fs_info, &it))
#else
	if (ipcufs_transfer(IFSA_UMOUNTING, NULL, 0))
#endif
		pr_err("%s: problem sending umount\n", __func__);

	kfree(sb->s_fs_info);
	sb->s_fs_info = NULL;
}

static struct super_operations ipcufs_ops = {
	.drop_inode = generic_delete_inode,
	.umount_begin = ipcufs_umount_begin,
};

static int ipcufs_fill_sb(struct super_block *sb, void *data, int silent)
{
	struct inode *root;
	const char *p = data;

	if (p) {
		while (*p && *p != ',' && *p != '=')
			p++;
		if (*p != '=') {
			pr_info("%s: mount option fs=<name> needed\n",
				__func__);
			return -EINVAL;
		}
		p++;
		sb->s_fs_info = kstrdup((const char *)p, GFP_KERNEL);
	}

	sb->s_op = &ipcufs_ops;
	sb->s_maxbytes = 0x7fffffffffffffffLL;

	sb->s_blocksize = 4096;
	sb->s_blocksize_bits = 12;

	root = ipcufs_iget(sb, 1);
	sb->s_root = d_make_root(root);
	if (!sb->s_root)
		return -EINVAL;

	return 0;
}

static struct dentry *ipcufs_mount(struct file_system_type *fs_type,
				   int flags, const char *dev_name, void *data)
{
	struct ipcufs_transport it;
	const char *p = data;

	if (p) {
		while (*p && *p != ',' && *p != '=')
			p++;
		if (*p != '=') {
			pr_info("%s: mount option fs=<name> needed\n",
				__func__);
			return ERR_PTR(-EINVAL);
		}
		p++;
	} else
		p = "";

	it.action = IFSA_MOUNTING;
#ifdef IPCUFS_LOOPBACK
	/* superblock doesn't exist yet, use temp filesystem name copy */
	if (ipcufs_transfer_request(p, &it))
#else
	if (ipcufs_transfer(IFSA_MOUNTING, NULL, 0))
#endif
		return ERR_PTR(-EINVAL);

	return mount_ns(fs_type, flags, data, ipcufs_fill_sb);
}

static struct file_system_type ipcufs_type = {
	.name = "tipcufs",
	.mount = ipcufs_mount,
	.kill_sb = kill_anon_super,
	.fs_flags = FS_REQUIRES_DEV,
};

static int sn_ipcufs_probe(struct platform_device *pdev)
{
#ifdef IPCUFS_LOOPBACK
	return register_filesystem(&ipcufs_type);
#else
	struct device *dev = &pdev->dev;
	int ret;

	if (of_property_read_u32(dev->of_node, "rtos-mtp-if", &dest_cpu)) {
		dev_err(dev, "%s:%d No RTOS cpu i/f provided\n",
			__func__, __LINE__);
		return -EINVAL;
	}

	if (dest_cpu >= 16) {
		dev_err(dev, "Invalid destination cpu i/f %d\n", dest_cpu);
		return -EINVAL;
	}

	shm = dma_alloc_coherent(dev, PAGE_SIZE, &shm_paddr, GFP_KERNEL);
	if (!shm)
		return -ENOMEM;

	dest_cpu = BIT(dest_cpu);
	mutex_init(&mlock);
	cl.dev = dev;
	cl.rx_callback = ipcufs_callback;
	cl.tx_block = true;
	cl.tx_tout = 500;
	cl.knows_txdone = false;

	chan = mbox_request_channel(&cl, 0);
	if (IS_ERR_OR_NULL(chan)) {
		ret = -EAGAIN;
		goto err_mbox;
	}

	ret = register_filesystem(&ipcufs_type);
	if (ret)
		goto err_fs;

	return 0;
 err_fs:
	mbox_free_channel(chan);
 err_mbox:
	dma_free_coherent(dev, PAGE_SIZE, shm, shm_paddr);
	return ret;
#endif
}

static int sn_ipcufs_remove(struct platform_device *pdev)
{
	unregister_filesystem(&ipcufs_type);
#ifndef IPCUFS_LOOPBACK
	mbox_free_channel(chan);
	dma_free_coherent(&pdev->dev, PAGE_SIZE, shm, shm_paddr);
#endif
	return 0;
}

static const struct of_device_id sn_ipcufs_id[] = {
	{.compatible = "socionext,ipcufs-client"},
	{},
};

MODULE_DEVICE_TABLE(of, sn_ipcufs_id);

static struct platform_driver sn_ipcufs_driver = {
	.probe = sn_ipcufs_probe,
	.remove = sn_ipcufs_remove,
	.driver = {
		   .name = "ipcufs-client",
		   .of_match_table = sn_ipcufs_id,
		   },
};

module_platform_driver(sn_ipcufs_driver);

MODULE_LICENSE("GPL");
